/**
 * @file nF_error.h
 * @Author Cunman Zhang (cz@nanoFaktur.com)  
 * @date 14.11.2017
 * @brief DLL errors for nanoFaktur's controllers
 *
 * @copyright nanoFaktur GmbH, 2017, all right reserved
 *
 * Error code for DLL functions, the controller error should be checked with function nF_get_dev_error()
 *
 */
#ifndef _NF_DLL_ERROR_CODE_H_
#define _NF_DLL_ERROR_CODE_H_

#define NF_DLL_ERRCODE_NONE                  0       // No error
// Note: Error code between -1 ~ -1999 is from controller

#define NF_DLL_ERRCODE_CMD_PKG              -2000    // Invalid command input package
#define NF_DLL_ERRCODE_CMD_SEND             -2001    // Error by sending command package 
#define NF_DLL_ERRCODE_RSP_TIMEOUT          -2002    // Timeout by reading response 
#define NF_DLL_ERRCODE_PARAMETER_NUM        -2003    // Error number of parameters
#define NF_DLL_ERRCODE_PARAMETER_IN         -2004    // Input parameter error

#define NF_DLL_ERRCODE_FD_INVALID           -2010    // Invalid file descriptor
#define NF_DLL_ERRCODE_FD_OP                -2011    // file descriptor read/write error

#define NF_DLL_ERRCODE_TCPIP_BUSY           -2020    // TCPIP interface is busy (interface is already connected)
#define NF_DLL_ERRCODE_IPADDR_ERROR         -2021    // IP address error

#define NF_DLL_ERRCODE_TCPIP_INIT           -2910    // system error: initialize TCPIP, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_TCPIP_SEND           -2911    // system error: TCPIP interface sending, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_TCPIP_RCV            -2912    // system error: TCPIP interface receiving, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_UDP_INIT             -2914    // system error: initialize UDP, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_UDP_SEND             -2915    // system error: UDP interface sending, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_UDP_RCV              -2916    // system error: UDP interface receiving, call nF_get_sys_last_error()

#define NF_DLL_ERRCODE_TIMER                -2920    // system error: timer, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_TIMER_ID             -2921    // Timer ID not found
#define NF_DLL_ERRCODE_TASK                 -2922    // system error: task, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_TASK_ADD             -2923    // Error by adding a task
#define NF_DLL_ERRCODE_MUTEX                -2924    // system error: mutex, call nF_get_sys_last_error()
#define NF_DLL_ERRCODE_MALLOC               -2925    // system error: malloc, call nF_get_sys_last_error()

#define NF_DLL_ERRCODE_FUNC_INVALID         -3000    // function not implemented yet

#endif // _NF_DLL_ERROR_CODE_H_
