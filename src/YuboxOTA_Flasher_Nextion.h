#ifndef _YUBOX_OTA_FLASHER_NEXTION_TFT_H_
#define _YUBOX_OTA_FLASHER_NEXTION_TFT_H_

#include <YuboxOTA_Flasher.h>
#include <HardwareSerial.h>

typedef std::function<void (bool) > YuboxOTA_Flasher_Nextion_TogglePause_func_cb;

class YuboxOTA_Flasher_Nextion : public YuboxOTA_Flasher
{
private:
    bool _uploadRejected;
    String _responseMsg;

    HardwareSerial * _pNexSerial;
    YuboxOTA_Flasher_Nextion_TogglePause_func_cb _togglepause_cb;
    uint32_t _old_baudrate;
    uint32_t _curr_baudrate;
    uint32_t _maxFlashSize;
    bool _fileIgnored;
    bool _flashing;
    bool _alreadyFlashed;
    bool _resetMade;

    uint32_t _totalUpload;
    uint32_t _currUpload;

    void _discardRxData(void);
    void _sendCommand(const char *, bool eoc = true, bool nh = false);
    uint16_t _recvString(String & resp, uint32_t timeout = 500, bool recv_flag = false);
    void _parseComOKResponse(String &);
    bool _testBaudrate(uint32_t baudrate);
    uint32_t _queryBaudrate(void);
    bool _echoTest(String);
    bool _desactivarSleepDim(void);
    uint32_t _calcTransmitTime(String &);
    bool _connect(void);
    bool _prepareFlashUpdate(uint32_t baudrate);

    void _dumpSerialData(bool, String &);

protected:
    virtual uint32_t _getBaudRate(void);
    virtual void _updateBaudRate(unsigned long);

public:
    YuboxOTA_Flasher_Nextion(void);
    ~YuboxOTA_Flasher_Nextion();

    // Called in order to setup everything for receiving update chunks
    bool startUpdate(void);

    // Called when stream EOF has been encountered but tar data has not finished (truncated file)
    void truncateUpdate(void);

    // Called when tar data stream has finished properly
    bool finishUpdate(void);

    // Check if current update process cannot continue due to an unrecoverable error
    bool isUpdateRejected(void);

    // Get last error message from update process, or empty string on success
    String getLastErrorMessage(void);

    // Start new file entry from uncompressed update stream
    bool startFile(const char *, unsigned long long);

    // Append data to file entry opened with startFile()
    bool appendFileData(const char *, unsigned long long, unsigned char *, int);

    // Finish data on opened file entry
    bool finishFile(const char *, unsigned long long);

    // Check if this firmware update needs a reboot to be applied
    bool shouldReboot(void);

    bool canRollBack(void);

    bool doRollBack(void);

    /*************************************************************************/

    void attachNextion(YuboxOTA_Flasher_Nextion_TogglePause_func_cb, HardwareSerial *);
};

#endif