#pragma once

#include <string>
#include <windows.h>
#include <tchar.h>

#ifdef _UNICODE
using _tstring = std::wstring;
#else
using _tstring = std::string;
#endif

class CSerialPort
{
public:
    CSerialPort();
    CSerialPort(const CSerialPort& r) = delete;
    ~CSerialPort();

    bool Open(
            int nComm,                      
            DWORD dwBaudRate = CBR_115200,  
            BYTE bByteSize = 8,             
            BYTE bParity = NOPARITY,        
            BYTE bStopBits = ONESTOPBIT     
    );

    bool Open(
            const _tstring& strName = _T("COM1"),        
            DWORD dwBaudRate = CBR_115200,  
            BYTE bByteSize = 8,             
            BYTE bParity = NOPARITY,        
            BYTE bStopBits = ONESTOPBIT     
    );

    bool IsOpen() const;

    bool SetState(
            DWORD dwBaudRate = CBR_115200,  
            BYTE bByteSize = 8,             
            BYTE bParity = NOPARITY,        
            BYTE bStopBits = ONESTOPBIT     
    );

    bool SetState(LPDCB lpDcb);

    bool SetupComm(
            DWORD dwInQueue = 14,   
            DWORD dwOutQueue = 16   
    );

    bool SetTimeOut(
            DWORD ReadInterval = 50,          
            DWORD ReadTotalMultiplier = 5,   
            DWORD ReadTotalConstant = 1000,            
            DWORD WriteTotalMultiplier = 10,         
            DWORD WriteTotalConstant = 200            
    );

    bool SetTimeOut(LPCOMMTIMEOUTS lpTimeOuts);

    bool Purge(
            DWORD  dwFlags = PURGE_TXCLEAR | PURGE_RXCLEAR
    );

    bool ClearError();

    void Close();

    bool Write(
            LPCVOID lpData,
            DWORD cbSize,
            LPDWORD lpBytesWritten = nullptr,
            DWORD nTimeOut = 1000
    );

    bool Read(
            LPVOID lpData,
            LPDWORD lpBytesRead = nullptr,
            DWORD nTimeOut = 3000
    );

private:

    bool OpenComm(int nComm);

    bool OpenComm(const _tstring& strName);

    bool WaitForOverlapped(
            HANDLE hFile,
            LPOVERLAPPED lpOv,
            LPDWORD lpBytesTransferred,
            DWORD nTimeOut
    );

private:

    HANDLE m_hComm;            
    HANDLE m_hReadEvent;        
    HANDLE m_hWriteEvent;       
    bool m_bExit;               
};