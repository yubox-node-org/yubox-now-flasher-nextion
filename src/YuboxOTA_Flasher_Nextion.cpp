#include <Arduino.h>

#include "YuboxOTA_Flasher_Nextion.h"

#include "esp_task_wdt.h"

#define NEX_UPLOAD_BAUDRATE 921600

YuboxOTA_Flasher_Nextion::YuboxOTA_Flasher_Nextion(void)
 : YuboxOTA_Flasher()
{
    _uploadRejected = false;
    _responseMsg = "";
    _pNexSerial = NULL;
    _old_baudrate = 0;
    _curr_baudrate = 0;
    _maxFlashSize = 0;
    _flashing = true;
    _fileIgnored = false;
    _alreadyFlashed = false;
    _resetMade = false;
}

bool YuboxOTA_Flasher_Nextion::isUpdateRejected(void)
{
    return _uploadRejected;
}

String YuboxOTA_Flasher_Nextion::getLastErrorMessage(void)
{
    return _responseMsg;
}

bool YuboxOTA_Flasher_Nextion::canRollBack(void)
{
    return false;
}

bool YuboxOTA_Flasher_Nextion::shouldReboot(void)
{
    return false;
}

bool YuboxOTA_Flasher_Nextion::doRollBack(void)
{
    _responseMsg = "Rollback de firmware de Nextion no está implementado.";
    return false;
}

void YuboxOTA_Flasher_Nextion::attachNextion(YuboxOTA_Flasher_Nextion_TogglePause_func_cb tp_cb, HardwareSerial * sp)
{
    _togglepause_cb = tp_cb;
    _pNexSerial = sp;
}

void YuboxOTA_Flasher_Nextion::_discardRxData(void)
{
    while (_pNexSerial->available()) _pNexSerial->read();
}

void YuboxOTA_Flasher_Nextion::_sendCommand(const char * cmd, bool eoc, bool nh)
{
    _discardRxData();
    if (nh) _pNexSerial->write(0x00);
    _pNexSerial->print(cmd);
    if (eoc) _pNexSerial->write("\xff\xff\xff");
    //Serial.printf("DEBUG: <-- %s\r\n", cmd);
}

uint16_t YuboxOTA_Flasher_Nextion::_recvString(String & resp, uint32_t timeout, bool recv_flag)
{
    uint32_t t = millis();
    uint8_t numEOC = 0;
    bool exitFlag = false;

    resp.clear();
    do {
        while (_pNexSerial->available()) {
            int val = _pNexSerial->read();
            if (val < 0) continue;  // <-- -1 indica que no hay datos disponibles
            //if (val == 0) continue;

            resp += (char)val;

            if (val == 0xFF) numEOC++; else numEOC = 0;
            if (recv_flag && resp.indexOf(0x05) != -1) exitFlag = true;
        }
    } while (numEOC < 3 && !exitFlag && millis() - t <= timeout);

    if (numEOC >= 3) resp = resp.substring(0, resp.length() - 3);

    return resp.length();
}

void YuboxOTA_Flasher_Nextion::_parseComOKResponse(String & r)
{
    // comok 1,30601-0,NX3224T028_011R,142,61488,DE6908C0D3473F23,4194304
    int idxSep[6];
    
    for (auto i = 0; i < 6; i++) {
        idxSep[i] = r.indexOf(',', (i > 0) ? (idxSep[i-1] + 1) : 0);
        if (idxSep[i] == -1) {
            Serial.printf("ERR: _parseComOKResponse() no encuentra coma #%d\r\n", i);
            return;
        }
    }
    String f;

    //f = r.substring(idxSep[1] + 1, idxSep[2]);
    //Serial.printf("DEBUG: _parseComOKResponse() modelo es %s\r\n", f.c_str());

    f = r.substring(idxSep[5] + 1);
    //Serial.printf("DEBUG: _parseComOKResponse() max flash (bytes) es %s\r\n", f.c_str());
    long v = f.toInt();
    if (v <= 0) {
        Serial.printf("ERR: _parseComOKResponse() no reconoce como número el valor: %s\r\n", f.c_str());
    } else {
        _maxFlashSize = v;
    }
}

bool YuboxOTA_Flasher_Nextion::_testBaudrate(uint32_t baudrate)
{
    String r;

    //Serial.printf("DEBUG: probando Nextion con velocidad: %d ...\r\n", baudrate);
    _pNexSerial->flush();
    _pNexSerial->updateBaudRate(baudrate);

    //Serial.println("DEBUG: se intenta obtener información de la pantalla...");
    _sendCommand("DRAKJHSUYDGBNCJHGJKSHBDN");
    _sendCommand("", true, true);   // 0x00 0xFF 0xFF 0xFF

    _recvString(r);
    if (r.length() > 0 && r[0] == 0x1a) {
        //Serial.println("...se obtiene respuesta, baudrate podría ser correcto.");
    } else {
        //Serial.println("...no hay respuesta coherente, baudrate parece incorrecto.");
    }

    //Serial.printf("DEBUG: conectando (intento 1 a %d bps)...\r\n", baudrate);
    _sendCommand("connect"); _recvString(r);
    if (r.indexOf("comok") != -1) {
        //Serial.printf("...respuesta es: %s\r\n", r.c_str());
        _parseComOKResponse(r);
    } else {
        //Serial.println("...no hay respuesta coherente, baudrate parece incorrecto.");
    }

    delay(110);
    _sendCommand("\xff\xff", false);

    //Serial.printf("DEBUG: conectando (intento 2 a %d bps)...\r\n", baudrate);
    _sendCommand("connect"); _recvString(r);
    if (r.indexOf("comok") != -1 || r[0] == 0x1a) {
        //Serial.printf("...respuesta es: %s\r\n", r.c_str());
        if (r.indexOf("comok") != -1) _parseComOKResponse(r);
        return true;
    } else {
        //Serial.println("...no hay respuesta coherente, baudrate parece incorrecto.");
        return false;
    }
}

uint32_t YuboxOTA_Flasher_Nextion::_queryBaudrate(void)
{
    // 250000 se usa en OpenVenti
    // 115200 se usa en otros proyectos
    // 9600 es la velocidad por omisión Nextion
    uint32_t baudrates[] = { 250000, 115200, 9600, 921600, 512000, 256000, 230400, 57600, 38400, 31250, 19200, 4800, 2400, 0 };

    // Probar primero con el valor de _old_baudrate para aprovechar
    // negociación inicial de interfaz YUBOX.
    if (_old_baudrate != 0) {
        if (_testBaudrate(_old_baudrate)) {
            //Serial.printf("DEBUG: prueba exitosa con baudrate (de trabajo) %d!\r\n", _old_baudrate);
            return _old_baudrate;
        }
        esp_task_wdt_reset();
    }

    // La negociación con el baudrate de trabajo de YUBOX ha fallado, hay que
    // intentar todos los otros baudrates hasta que uno funcione.
    for (auto i = 0; baudrates[i] > 0; i++) {
        if (_testBaudrate(baudrates[i])) {
            //Serial.printf("DEBUG: prueba exitosa con baudrate %d!\r\n", baudrates[i]);
            return baudrates[i];
        }
        esp_task_wdt_reset();
        delay(1500);
    }
    return 0;
}

uint32_t YuboxOTA_Flasher_Nextion::_calcTransmitTime(String & s)
{
    return (((1000000 * 10) / _curr_baudrate) * (s.length() + 3)) / 1000;
}

bool YuboxOTA_Flasher_Nextion::_echoTest(String s)
{
    String cmd = "print \"" + s + "\"";
    _sendCommand(cmd.c_str());

    uint32_t roundtrip_ms = _calcTransmitTime(cmd) * 2 + 10;

    String r;
    _recvString(r, roundtrip_ms);

    return (r.indexOf(s) != -1);
}

bool YuboxOTA_Flasher_Nextion::_connect(void)
{
    uint32_t queriedBaudrate = _queryBaudrate();
    if (queriedBaudrate == 0) {
        _responseMsg = "No se puede negociar velocidad de comunicación con Nextion, o el cableado está dañado.";
        _uploadRejected = true;
        return false;
    }

    _curr_baudrate = queriedBaudrate;

    // Establecer modo de ejecución
    delay(100);
    _sendCommand("runmod=2");
    delay(60);

    if (!_echoTest("YUBOX NEXTION UPDATE")) {
        _responseMsg = "Fallo en verificación de prueba de eco.";
        _uploadRejected = true;
        return false;
    }

    // TODO: handle sleep/dim

    return true;
}

#define CHECK_INITIALIZED() \
    if (_pNexSerial == NULL) {\
        _responseMsg = "(internal) Flasheador Nextion no inicializado con puerto serial!";\
        return false;\
    }

bool YuboxOTA_Flasher_Nextion::startUpdate(void)
{
    CHECK_INITIALIZED()

    // Se indica a interfaz ordinaria que detenga toda la interacción
    _togglepause_cb(true);

    _alreadyFlashed = false;
    _flashing = false;
    _fileIgnored = false;
    _resetMade = false;

    _old_baudrate = _pNexSerial->baudRate();
    if (_curr_baudrate == 0) _curr_baudrate = _old_baudrate;

    if (!_connect()) return false;

    return true;
}

void YuboxOTA_Flasher_Nextion::truncateUpdate(void)
{
    _responseMsg = "truncateUpdate: no implementado";
    _uploadRejected = true;
}

bool YuboxOTA_Flasher_Nextion::finishUpdate(void)
{
    CHECK_INITIALIZED()

    return !_uploadRejected;
}

bool YuboxOTA_Flasher_Nextion::startFile(const char * filename, unsigned long long filesize)
{
    CHECK_INITIALIZED()

    if (_alreadyFlashed) {
        // Entrada de archivo aceptada, pero se ignora
        //Serial.printf("DEBUG: se ignora entrada de archivo porque ya se flasheó firmware Nextion: %s\r\n", filename);
        return true;
    }

    // Verificar si el archivo proporcionado debe flashearse al Nextion
    _fileIgnored = false;
    const char * exten = strstr(filename, ".tft");
    if (exten == NULL || strlen(exten) != 4) {
        // Entrada de archivo aceptada, pero se ignora
        //Serial.printf("DEBUG: se ignora entrada de archivo porque no es firmware Nextion: %s\r\n", filename);
        _fileIgnored = true;
        return true;
    }

    // Verificar si el firmware proporcionado cabe en el flash de esta Nextion
    if (_maxFlashSize > 0 && filesize > _maxFlashSize) {
        _responseMsg = "Firmware demasiado grande para flash de dispositivo";
        _uploadRejected = true;
        return false;
    }

    _totalUpload = filesize;
    _currUpload = 0;

    uint32_t uploadSpeeds[] = {921600, 115200, 0};
    for (auto i = 0; uploadSpeeds[i] != 0; i++) {
        _responseMsg.clear();
        if (_prepareFlashUpdate(uploadSpeeds[i])) {
            _flashing = true;
            _filestart_cb(filename, true, filesize);
            return true;
        }
    }

    _responseMsg = "No se puede iniciar flasheo de firmware";
    _uploadRejected = true;
    return false;
}

bool YuboxOTA_Flasher_Nextion::_prepareFlashUpdate(uint32_t baudrate)
{
    String r;
    _sendCommand("00");
    delay(1);
    _recvString(r, 800, true);

    String cmd = "whmi-wri "; cmd += _totalUpload; cmd += ","; cmd += baudrate; cmd += ",0";
    _sendCommand(cmd.c_str());
    _pNexSerial->flush();
    //Serial.printf("DEBUG: _prepareFlashUpdate() se cambia velocidad a %d...\r\n", baudrate);
    _pNexSerial->updateBaudRate(baudrate);
    _recvString(r, 800, true);
    if (r.indexOf(0x05) != -1) {
        // Nextion ha aceptado la velocidad y tamaño del firmware
        return true;
    }

    // Recuperación: restaurar velocidad previa
    _pNexSerial->flush();
    //Serial.printf("DEBUG: _prepareFlashUpdate() se restaura velocidad a %d debido a error...\r\n", _curr_baudrate);
    _pNexSerial->updateBaudRate(_curr_baudrate);
    return false;
}

bool YuboxOTA_Flasher_Nextion::appendFileData(const char * filename, unsigned long long filesize, unsigned char * block, int size)
{
    CHECK_INITIALIZED()

    // Si la entrada actual está siendo ignorada, se acepta sin tomar acción
    if (_fileIgnored || _alreadyFlashed) return true;

    // Si el archivo actual no está siendo flasheado, se rechaza
    if (!_flashing) {
        if (_responseMsg.isEmpty()) _responseMsg = "(internal) Datos agregados luego de rechazar flasheo!";
        _uploadRejected = true;
        return false;
    }

    while (!_uploadRejected && size > 0 && _currUpload < _totalUpload) {
        _pNexSerial->write(*block);
        _currUpload++;
        size--; block++;

        if ((_currUpload & 0xfffU) == 0 || _currUpload >= _totalUpload) {
            auto retries = 0;
            do {
                String r;
                _recvString(r, 500, true);
                if (r.indexOf(0x05) != -1) {
                    //Serial.print(".");
                    break;
                }
                retries++;
                if (retries >= 8) {
                    _responseMsg = "Conexión serial perdida durante la actualización";
                    _uploadRejected = true;
                }
            } while (!_uploadRejected);
        }
    }
    _fileprogress_cb(filename, true, filesize, _currUpload);

    return !_uploadRejected;
}

bool YuboxOTA_Flasher_Nextion::finishFile(const char * filename, unsigned long long filesize)
{
    CHECK_INITIALIZED()

    // Si la entrada actual está siendo ignorada, se acepta sin tomar acción
    if (_fileIgnored || _alreadyFlashed) return true;

    // Si el archivo actual no está siendo flasheado, se rechaza
    if (!_flashing) {
        if (_responseMsg.isEmpty()) _responseMsg = "(internal) Datos agregados luego de rechazar flasheo!";
        _uploadRejected = true;
        return false;
    }
    _flashing = false;
    _alreadyFlashed = true;

    esp_task_wdt_reset();
    _fileend_cb(filename, true, filesize);

    if (!_uploadRejected) {
        // Esperar por paquete 0x88,0xFF,0xFF,0xFF
        uint32_t t = millis();
        String resp;

        _pNexSerial->flush();
        if (_old_baudrate != 0) _pNexSerial->updateBaudRate(_old_baudrate);

        while (millis() - t <= 2000) {
            if (_pNexSerial->available()) {
                int val = _pNexSerial->read();
                if (val < 0) continue;  // <-- -1 indica que no hay datos disponibles

                resp += (char)val;
                if (resp.length() >= 4 && resp.indexOf("\x88\xff\xff\xff") != -1) {
                    //Serial.println("DEBUG: encontrada respuesta 0x88! OK");
                    break;
                }
            }
        }
        esp_task_wdt_reset();
        if (resp.indexOf("\x88\xff\xff\xff") == -1) {
            //Serial.println(resp);
            //_responseMsg = "Upload no ha sido aceptado por Nextion, o timeout";
            //_uploadRejected = true;
        } else {
            _sendCommand("rest");
            _resetMade = true;
        }
    }

    return true;
}

YuboxOTA_Flasher_Nextion::~YuboxOTA_Flasher_Nextion()
{
    if (_pNexSerial != NULL && _old_baudrate != 0) {
        if (!_resetMade) {
            _pNexSerial->flush();
            _pNexSerial->updateBaudRate(_old_baudrate);
            _sendCommand("rest");
            delay(200);
        }
        _sendCommand("bkcmd=3");
        String r;
        _recvString(r);
    }
    _togglepause_cb(false);
}