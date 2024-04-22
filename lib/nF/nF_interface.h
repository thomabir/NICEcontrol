
/**
 * @file nF_interface.h
 * @Author Cunman Zhang (cz@nanoFaktur.com)  
 * @date 14.11.2017
 * @brief interface functions for nanoFaktur's controller
 *
 * @copyright nanoFaktur GmbH, 2017, all right reserved
 *
 * Interface function definition
 *
 */
#ifndef _NF_INTERFACE_DLL_H_
#define _NF_INTERFACE_DLL_H_

#include "nF_common.h" 
#include "nF_error.h" 

#define MAX_PACKAGE_LEN                 1024 // maximal comamnd/response package size
#define MAX_CMDIN_PARAMETER_NUM         128  // maximal number of parameters in command package (to controller)
#define MAX_RSP_PARAMETER_NUM           512  // maximal number of parameters in response package (from controller)
#define MAX_PARAMETER_STRLEN            32   // maximal string length for controller's paramaters

#define RSP_PACKAGE_END                 0    // response package end
#define RSP_PACKAGE_MORE                1    // controller has more response packages to send

typedef struct tagCommandCfg {
    u16 cmdId;       // Command ID: 2-Bytes
    u16 customId;    // Custom flag: 2-Bytes (not used by controller)
    u8 option;       // Options: e.g. acknowledge / read-write
    int dataNum;     // Number of input data
    u8 *pDataFmt;    // Data/parameter format: e.g. CMD_DATA_FMT_FLOAT or CMD_DATA_FMT_U32 (see. nF_common.h)
    u32 *pDataIn;    // Data/parameter input for command
} CommandCfg;

typedef struct tagResponseMsg {
    u16 cmdId;       // ID of last command
    u16 customId;    // Custom flag
    int seq;         // response sequence
    u8 *pPackage;    // response package from controller
    int dataNum;     // number of data from controller
    u8 *pDataFmt;    // Data format
    u32 *pDataOut;   // Buffer for saving response data+
} ResponseMsg; // Note: call nF_free_rspMsg() to free the struct

typedef union tagParameterValue { // Parameter value to controller
    //double dData;  // not implemented yet
    u32 uData;       // unsigned-int
    float fData;     // float
    int nData;       // integer 
    char sData[MAX_PARAMETER_STRLEN]; // space reserved for string
} ParamVal;

//------------------------------------------
//  struct for data-recorder (from rev.3.40)
//------------------------------------------
typedef struct tagRecoderMemCfg { // Struct for configuring data Recorder memory
    u32 TableNum_A;  // Number of tables in group-A
    u32 TableSize_A; // Size of table in group-A
    u32 TableNum_B;  // Number of tables in group-B
    u32 TableSize_B; // Size of table in group-B
    int EvtNo;       // Event used (default=1)
} RecoderMemCfg; 

typedef struct tagRecoderCfg { // Struct for configuring data Recorder
    u32 rate;        // recorder rate
    int size;        // size of data (<=512 for EBD-0602x0)
    int num;         // number of items (<=2 for EBD-0602x0, <=16 for High-End)
    int rec[16];     // index of recorder table
    int src[16];     // data source
    int ch[16];      // data from which channel
} RecoderCfg; 

//------------------------------------------
//  struct for function call (from rev.3.70)
//------------------------------------------
typedef struct tagParamX {
    int fd;          // interface file-descriptor
    u16 customId;    // Custom flag
} ParamX;

#ifdef WIN32
	#ifdef BUILD_DLL_API    // not for custom
	#define _DLL_API_ __declspec(dllexport)
	#else
	#define _DLL_API_ __declspec(dllimport)
	#endif
#else // for Linux
	#define _DLL_API_ extern
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get DLL software revision
 * @param None
 * @return Revision of the DLL
*/
float _DLL_API_ nF_get_dll_revision(void);

/**
 * @brief Get DLL error code
 * @param None
 * @return Error code of the DLL
*/
int _DLL_API_ nF_get_dll_last_error(void);

/**
 * @brief Get error code by calling windows-system functions
 * @param None
 * @return System error code (see https://msdn.microsoft.com/de-de/library/windows/desktop/ms681381(v=vs.85).aspx)
*/
int _DLL_API_ nF_get_sys_last_error(void);

/**
 * @brief Set DLL logging status
 * @param[in] log status controlling (0: no logging, 1: logging enabled)
 * @return 0=successful, otherwise=failed
 * @note: when enabled, information will be saved in file "nF_interface_log.txt"
*/
int _DLL_API_ nF_set_log_state(int enable);

/**
 * @brief Get DLL logging status
 * @param None
 * @return log status (0: no logging, 1: logging enabled)
*/
int _DLL_API_ nF_get_log_state(void);

/**
 * @brief Set DLL error-logging status
 * This is a simplified version of DLL-logging, i.e. saves information only when error occures
 * @param[in] log status controlling (0: no error-logging, 1: error-logging enabled)
 * @return 0=successful, otherwise=failed
 * @note: when enabled, information will be saved in file "nF_interface_errLog.txt"
*/
int _DLL_API_ nF_set_errLog_state(int enable);

/**
 * @brief Get DLL error-logging status
 * @param None
 * @return log status (0: no error-logging, 1: error-logging enabled)
*/
int _DLL_API_ nF_get_errLog_state(void);

/**
 * @brief Set task-delay-time for interface (speed vs cpu-usage)
 * @param[in] timedelay_ms value in millisecond
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_set_task_delay(int timedelay_ms);

/**
 * @brief Get task-delay-time for interface
 * @return value in millisecond ( when <0: error code)
*/
int _DLL_API_ nF_intf_get_task_delay(void);

/**
 * @brief Set timeout for interface
 * @param[in] fd interface file-descriptor
 * @param[in] timeout_ms timeout value in millisecond
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_set_timeout(int fd, int timeout_ms);

/**
 * @brief Get timeout for interface
 * @param[in] fd interface file-descriptor
 * @return timeout value in millisecond ( when <0: error code)
*/
int _DLL_API_ nF_intf_get_timeout(int fd);

/**
 * @brief Send/recieve UDP message for nanoFaktur's controllers
 * @param[in,out] pRes buffer for saving response
 * @param[in] bufLen buffer length
 * @param[in] timeout_ms timeout value in millisecond for waiting response
 * @return 0=successful, otherwise=failed
 * @note nanoFaktur controller uses port 51000
*/
int _DLL_API_ nF_udp_search(char *pRes, int bufLen, int timeout_ms);

/**
 * @brief Convert UDP message to valid IP-Address
 * @param[in] pUDPRsp response from the function nF_udp_search()
 * @param[in,out] pIPAddr buffer for saving IP address
 * @param[in] bufLen buffer length of pIPAddr (should >= 16)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_udp_rsp_to_ipaddr(const char *pUDPRsp, char *pIPAddr, int buflen);

/**
 * Not implemented yet
*/
int _DLL_API_ nF_intf_connect_dialog(void* pCfg);

/**
 * Not implemented yet
*/
int _DLL_API_ nF_intf_connect(void* pCfg);

/**
 * @brief connect to local port (simulation)
 * @param[in] name name of the simulated controller
 * @return interface file-descriptor, >=0:successful, otherwise=failed
 * @note run the controller simulation software first
*/
int _DLL_API_ nF_intf_connect_local(char *name);

/**

 * @brief connect to COM port (RS232)
 * @param[in] portNo COM port number (e.g.: 1 for COM1 [Windows], or ttyS1 [linux])
 * @param[in] portName COM port string (e.g.: /dev/ttyS1), for linux only
 * @return interface file-descriptor, >=0:successful, otherwise=failed
 * @note nanoFaktur controller uses the following configuration: 
 *     baudrate=115200, 
 *     no flow control
 *     8 Bits data / 1-stop-bit
 *     Parity = none
*/
int _DLL_API_ nF_intf_connect_com(int portNo);
int _DLL_API_ nF_intf_connect_tty(char *portName);

/**
 * @brief connect TCP/IP interface
 * @param[in] ipaddr A NULL-terminated IP-Address string (e.g. "192.168.178.1")
 * @return interface file-descriptor, >=0:successful, otherwise=failed
 * @note nanoFaktur controller uses port 51000
*/
int _DLL_API_ nF_intf_connect_tcpip(char *ipaddr);

/**
 * @brief disconnect interface
 * @param[in] fd interface file-descriptor
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_disconnect(int fd);

/**
 * @brief clear interface
 * @param[in] fd interface file-descriptor
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_clear(int fd);

/**
 * @brief send command to controller
 * @param[in] fd interface file-descriptor
 * @param[in] pCmdCfg Pointer of CommandCfg struct
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_write_command(int fd, CommandCfg *pCmdCfg);

/**
 * @brief send command to controller and then read response
 * @param[in] fd interface file-descriptor
 * @param[in] pCmdCfg Pointer of CommandCfg struct
 * @param[out] ppRspMsg Pointer of response message (must be free-ed by nF_free_rspMsg() )
 * @return RSP_PACKAGE_END=successful, RSP_PACKAGE_MORE=more data follow, otherwise=failed
*/
int _DLL_API_ nF_intf_read_command(int fd, CommandCfg *pCmdCfg, ResponseMsg **ppRspMsg);

/**
 * @brief read more response from controller
 * @param[in] fd interface file-descriptor
 * @param[out] ppRspMsg Pointer of response message (must be free-ed by nF_free_rspMsg() )
 * @return RSP_PACKAGE_END=successful, RSP_PACKAGE_MORE=more data follow, otherwise=failed
*/
int _DLL_API_ nF_intf_read_more(int fd, ResponseMsg **ppRspMsg);

/**
 * @brief free the response message 
 * @param[in] pRspMsg pointer of the response message  
 * @return none
 * @note should be called after nF_intf_read_command()/nF_intf_read_more()
*/
void _DLL_API_ nF_free_rspMsg(ResponseMsg *pRspMsg);

/**
 * @brief example function for showing the response
 * @param[in] fmt data format
 * @param[in] pData response data address 
 * @return none
*/
void _DLL_API_ nF_show_value(u8 fmt, void *pData);

/**
 * @brief Set default interface file-descriptor for "TOP-level" functions
 * @param[in] fd file-descriptor returned from nF_intf_connect_xxx() functions
 * @return 0=successful, otherwise=failed
 * @note: the first opened interface will be take as default.
 * @note: behavior by invalid input is undefined.
*/
int _DLL_API_ nF_intf_set_default_fd(int fd); 

/**
 * @brief Get the default interface file-descriptor
 * @param None
 * @return file-descriptor 
*/
int _DLL_API_ nF_intf_get_default_fd(void); 

//-----------------------------------------------------
// wrap-functions (similar to nF_intf_write_command() 
//       but with simple input parameters).
// Note: these functions use interface of the
//       default-file-descriptor
// Note: function *_m() has file-descriptor input
// Note: function *_mc() has file-descriptor and customId input
//-----------------------------------------------------
/**
 * @brief send command to controller (data format is u32)
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param[in] cmdId command-id to controller
 * @param[in] dataNum number of input parameters
 * @param[in] pVal value of input parameters
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_write_command_u32(int cmdId, int dataNum, u32 *pVal);
int _DLL_API_ nF_intf_write_command_u32_m(int fd, int cmdId, int dataNum, u32 *pVal);
int _DLL_API_ nF_intf_write_command_u32_x(ParamX paramX, int cmdId, int dataNum, u32 *pVal);

/**
 * @brief send command to controller (data format is float)
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param[in] cmdId command-id to controller
 * @param[in] dataNum number of input parameters
 * @param[in] pVal value of input parameters
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_intf_write_command_float(int cmdId, int dataNum, float *pVal);
int _DLL_API_ nF_intf_write_command_float_m(int fd, int cmdId, int dataNum, float *pVal);
int _DLL_API_ nF_intf_write_command_float_x(ParamX paramX, int cmdId, int dataNum, float *pVal);

/**
 * @brief send command to controller and then read response (data format is u32)
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param[in] cmdId command-id to controller
 * @param[in] cmdInNum number of input parameters
 * @param[in] pCmdInVal value of input parameters
 * @param[out] pRspNum Pointer of response number
 * @param[out] pRsp Pointer of response message
 * @return 0=successful, otherwise=failed
 * example: nF_intf_read_command_u32(0x1000, 0, NULL, &len, &data[0]);
*/
int _DLL_API_ nF_intf_read_command_u32(int cmdId, int cmdInNum, u32 *pCmdInVal, int *pRspNum, u32 *pRsp);
int _DLL_API_ nF_intf_read_command_u32_m(int fd, int cmdId, int cmdInNum, u32 *pCmdInVal, int *pRspNum, u32 *pRsp);
int _DLL_API_ nF_intf_read_command_u32_x(ParamX paramX, int cmdId, int cmdInNum, u32 *pCmdInVal, int *pRspNum, u32 *pRsp);

/**
 * @brief read one value from controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param[in] cmdId command-id to controller
 * @param[in] index axis/channel index (set to -1 when command has no index)
 * @param[out] pVal Pointer of axis/channel value
 * @return 0=successful, otherwise=failed
 * example: nF_intf_read_value1(0x2111, 0, &data);
*/
int _DLL_API_ nF_intf_read_value1(int cmdId, int index, float *pVal);
int _DLL_API_ nF_intf_read_value1_m(int fd, int cmdId, int index, float *pVal);
int _DLL_API_ nF_intf_read_value1_x(ParamX paramX, int cmdId, int index, float *pVal);

//-----------------------------------------------------
// TOP-level functions
// Implemented the most often used functions
// Note: these functions use interface of the
//       default-file-descriptor
// Note: function *_m() has file-descriptor input
//-----------------------------------------------------
/**
 * @brief Get the error code of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param None
 * @return error code
*/
int _DLL_API_ nF_get_dev_error(void); 
int _DLL_API_ nF_get_dev_error_m(int fd); 
int _DLL_API_ nF_get_dev_error_x(ParamX param); 

/**
 * @brief Get axis-position of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_position(u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_position_m(int fd, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_position_x(ParamX paramX, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get On-Target status of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data (value 0: not-On-Target, 1: On-Target)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_ont(u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_ont_m(int fd, u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_ont_x(ParamX paramX, u8 num, int *pAxis, int *pVal);

/**
 * @brief Get Overflow status of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data (value 0: not-overflow, 1: overflow)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_ovf(u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_ovf_m(int fd, u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_ovf_x(ParamX paramX, u8 num, int *pAxis, int *pVal);

/**
 * @brief Get servo status of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data (value 0: open-loop, 1: close-loop)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_svo(u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_svo_m(int fd, u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_get_dev_axis_svo_x(ParamX paramX, u8 num, int *pAxis, int *pVal);

/**
 * @brief Set servo controlling of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Input data (value 0: open-loop, 1: close-loop)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_axis_svo(u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_set_dev_axis_svo_m(int fd, u8 num, int *pAxis, int *pVal);
int _DLL_API_ nF_set_dev_axis_svo_x(ParamX paramX, u8 num, int *pAxis, int *pVal);

/**
 * @brief Get control-voltage of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_cVol(u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_cVol_m(int fd, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_cVol_x(ParamX paramX, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get current using target of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_target_use(u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_target_use_m(int fd, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_target_use_x(ParamX paramX, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get target of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value from close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_target(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_target_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_target_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Set target of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value for close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Input data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_axis_target(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_target_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_target_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get low-soft-limit (in use) of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value from close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_range_low(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_range_low_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_range_low_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Set low-soft-limit (in use) of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value for close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_axis_range_low(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_range_low_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_range_low_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get high-soft-limit (in use) of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value from close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_axis_range_high(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_range_high_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_get_dev_axis_range_high_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Set high-soft-limit (in use) of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param bSvo value for close-loop (when 1) or open-loop (when 0)
 * @param num Number of channels (1 ~ 3)
 * @param pAxis Channel index (0, 1, 2...) 
 * @param pVal Response data
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_axis_range_high(u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_range_high_m(int fd, u8 bSvo, u8 num, int *pAxis, float *pVal);
int _DLL_API_ nF_set_dev_axis_range_high_x(ParamX paramX, u8 bSvo, u8 num, int *pAxis, float *pVal);

/**
 * @brief Get RAM (volatile) parameter of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Channel index (0, 1, 2...) 
 * @param id Parameter index  
 * @param pFmt Response data format
 * @param pVal Response data (see union ParamVal)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_parameter_ram(int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_ram_m(int fd, int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_ram_x(ParamX paramX, int ch, u32 id, u8 *pFmt, ParamVal *pVal);

/**
 * @brief Set RAM (volatile) parameter of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Channel index (0, 1, 2...) 
 * @param id Parameter index  
 * @param fmt Input data format
 * @param pVal Input data (see union ParamVal)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_parameter_ram(int ch, u32 id, u8 fmt, ParamVal *pVal);
int _DLL_API_ nF_set_dev_parameter_ram_m(int fd, int ch, u32 id, u8 fmt, ParamVal *pVal);
int _DLL_API_ nF_set_dev_parameter_ram_x(ParamX paramX, int ch, u32 id, u8 fmt, ParamVal *pVal);

/**
 * @brief Get Flash (no-volatile) parameter of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Channel index (0, 1, 2...) 
 * @param id Parameter index  
 * @param pFmt Response data format
 * @param pVal Response data (see union ParamVal)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_parameter_flash(int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_flash_m(int fd, int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_flash_x(ParamX paramX, int ch, u32 id, u8 *pFmt, ParamVal *pVal);

/**
 * @brief Set Flash (no-volatile) parameter of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Channel index (0, 1, 2...) 
 * @param id Parameter index  
 * @param fmt Input data format
 * @param pVal Input data (see union ParamVal)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_parameter_flash(int ch, u32 id, u8 fmt, ParamVal *pVal);
int _DLL_API_ nF_set_dev_parameter_flash_m(int fd, int ch, u32 id, u8 fmt, ParamVal *pVal);
int _DLL_API_ nF_set_dev_parameter_flash_x(ParamX paramX, int ch, u32 id, u8 fmt, ParamVal *pVal);

/**
 * @brief Get (factory) default parameter of controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Channel index (0, 1, 2...) 
 * @param id Parameter index  
 * @param pFmt Response data format
 * @param pVal Response data (see union ParamVal)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_parameter_default(int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_default_m(int fd, int ch, u32 id, u8 *pFmt, ParamVal *pVal);
int _DLL_API_ nF_get_dev_parameter_default_x(ParamX paramX, int ch, u32 id, u8 *pFmt, ParamVal *pVal);

//-----------------------------------------------------------------------------------
//  Functions for data-recorder
//-----------------------------------------------------------------------------------
// Example for calling these functions:
//        nF_set_dev_rec_mem_cfg(recMemCfg); // optional
//        nF_set_dev_rec_cfg(recCfg);
//        nF_start_dev_rec();
//        nF_get_dev_rec_length();
//        nF_get_dev_rec_data(...);

/**
 * @brief get data recorder memory's configuration
 * @param pRecMemCfg buffer-address for reading back (see struct RecoderMemCfg)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_rec_mem_cfg(RecoderMemCfg *pRecMemCfg);

/**
 * @brief Configure data recorder memory
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param recMemCfg Memory configuration of recorder (see struct RecoderMemCfg)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_rec_mem_cfg(RecoderMemCfg recMemCfg);
int _DLL_API_ nF_set_dev_rec_mem_cfg_m(int fd, RecoderMemCfg recMemCfg);
int _DLL_API_ nF_set_dev_rec_mem_cfg_x(ParamX paramX, RecoderMemCfg recMemCfg);

/**
 * @brief get data recorder configuration
 * @param pRecCfg buffer-address for reading back (see struct RecoderCfg)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_rec_cfg(RecoderCfg *pRecCfg);

/**
 * @brief Configure data recorder
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param recCfg configuration of recorder (see struct RecoderCfg)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_set_dev_rec_cfg(RecoderCfg recCfg);
int _DLL_API_ nF_set_dev_rec_cfg_m(int fd, RecoderCfg recCfg);
int _DLL_API_ nF_set_dev_rec_cfg_x(ParamX paramX, RecoderCfg recCfg);

/**
 * @brief Start to recorder data
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_start_dev_rec(void);
int _DLL_API_ nF_start_dev_rec_m(int fd);
int _DLL_API_ nF_start_dev_rec_x(ParamX paramX);

/**
 * @brief Get valid recorder data length from controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch recorder-group index (0, 1) 
 * @return valid data length in recorder-table (failed when < 0)
*/
int _DLL_API_ nF_get_dev_rec_length(int ch);
int _DLL_API_ nF_get_dev_rec_length_m(int fd, int ch);
int _DLL_API_ nF_get_dev_rec_length_x(ParamX paramX, int ch);

/**
 * @brief Get recorder data from controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param recCh recorder channel index (0, 1, 2...) 
 * @param from memory starting address (0, ... len-1)
 * @param size number of data to get
 * @param pBuf Response data (buffer must be valid)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_rec_data(int recCh, u32 from, u32 size, float *pBuf);
int _DLL_API_ nF_get_dev_rec_data_m(int fd, int recCh, u32 from, u32 size, float *pBuf);
int _DLL_API_ nF_get_dev_rec_data_x(ParamX paramX, int recCh, u32 from, u32 size, float *pBuf);

//-----------------------------------------------------------------------------------
//  Functions for wave-generator (for High-End controllers only)
//-----------------------------------------------------------------------------------
/**
 * @brief Get wave-table data from controller
 * @param[in] fd interface file-descriptor for function _m()
 * @param[in] ParamX parameter input for function *_x()
 * @param ch Wave index (0, 1, 2...) 
 * @param from memory starting address (0, ... len-1)
 * @param size number of data to get
 * @param pBuf Response data (buffer must be valid)
 * @return 0=successful, otherwise=failed
*/
int _DLL_API_ nF_get_dev_wave_data(int ch, u32 from, u32 size, float *pBuf);
int _DLL_API_ nF_get_dev_wave_data_m(int fd, int ch, u32 from, u32 size, float *pBuf);
int _DLL_API_ nF_get_dev_wave_data_x(ParamX paramX, int ch, u32 from, u32 size, float *pBuf);

#ifdef __cplusplus
}
#endif

#endif /* _NF_INTERFACE_DLL_H_ */
