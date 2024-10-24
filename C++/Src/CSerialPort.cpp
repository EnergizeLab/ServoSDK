#include "CSerialPort.h"
#include <strsafe.h>

CSerialPort::CSerialPort()
        :
        m_hComm(INVALID_HANDLE_VALUE),
        m_hReadEvent(nullptr),
        m_hWriteEvent(nullptr),
        m_bExit(false)
{

}

CSerialPort::~CSerialPort()
{
    Close();
}

bool CSerialPort::Open(
        int nComm,
        DWORD dwBaudRate/* = CBR_115200*/,
        BYTE bByteSize/* = 8*/,
        BYTE bParity/* = NOPARITY*/,
        BYTE bStopBits/* = ONESTOPBIT*/
)
{
    if (!OpenComm(nComm))
    {
        return false;
    }

    if (!SetState(dwBaudRate, bByteSize, bParity, bStopBits))
    {
        this->Close();
        return false;
    }

    m_bExit = false;
    return true;
}

bool CSerialPort::Open(
        const _tstring& strName/* = _T("COM1")*/,           
        DWORD dwBaudRate/* = CBR_115200*/,
        BYTE bByteSize/* = 8*/,
        BYTE bParity/* = NOPARITY*/,
        BYTE bStopBits/* = ONESTOPBIT*/
)
{
    if (!OpenComm(strName))
    {
        return false;
    }

    if (!SetState(dwBaudRate, bByteSize, bParity, bStopBits))
    {
        this->Close();
        return false;
    }

    m_bExit = false;
    return true;
}

bool CSerialPort::IsOpen() const
{
    return INVALID_HANDLE_VALUE != m_hComm;
}

bool CSerialPort::SetState(
        DWORD dwBaudRate/* = CBR_115200*/,
        BYTE bByteSize/* = 8*/,
        BYTE bParity/* = NOPARITY*/,
        BYTE bStopBits/* = ONESTOPBIT*/
)
{
    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);

    if (!::GetCommState(m_hComm, &dcb))
    {
        return false;
    }

    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = dwBaudRate;
    dcb.ByteSize = bByteSize;
    dcb.Parity = bParity;
    dcb.StopBits = bStopBits;

    return ::SetCommState(m_hComm, &dcb);
}

bool CSerialPort::SetState(LPDCB lpDcb)
{
    if (nullptr == lpDcb)
    {
        return false;
    }

    return ::SetCommState(m_hComm, lpDcb);
}

bool CSerialPort::SetupComm(
        DWORD dwInQueue/* = 14*/,
        DWORD dwOutQueue/* = 16*/
)
{
    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    return ::SetupComm(m_hComm, dwInQueue, dwOutQueue);
}

bool CSerialPort::SetTimeOut(
        DWORD ReadInterval/* = 50*/,
        DWORD ReadTotalMultiplier/* = 50*/,
        DWORD ReadTotalConstant/* = 100*/,
        DWORD WriteTotalMultiplier/* = 50*/,
        DWORD WriteTotalConstant/* = 200*/
)
{
    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    COMMTIMEOUTS comTimeOuts = { 0 };

    if (!::GetCommTimeouts(m_hComm, &comTimeOuts))
    {
        return false;
    }

    comTimeOuts.ReadIntervalTimeout = ReadInterval;
    comTimeOuts.ReadTotalTimeoutMultiplier = ReadTotalMultiplier;
    comTimeOuts.ReadTotalTimeoutConstant = ReadTotalConstant;
    comTimeOuts.WriteTotalTimeoutMultiplier = WriteTotalMultiplier;
    comTimeOuts.WriteTotalTimeoutConstant = WriteTotalConstant;
    return ::SetCommTimeouts(m_hComm, &comTimeOuts);
}

bool CSerialPort::SetTimeOut(LPCOMMTIMEOUTS lpTimeOuts)
{
    if (nullptr == lpTimeOuts)
    {
        return false;
    }

    return ::SetCommTimeouts(m_hComm, lpTimeOuts);
}

bool CSerialPort::Purge(
        DWORD  dwFlags/* = PURGE_TXCLEAR | PURGE_RXCLEAR*/
)
{
    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    return ::PurgeComm(m_hComm, dwFlags);
}

bool CSerialPort::ClearError()
{
    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    DWORD dwError = 0;
    COMSTAT comStat = { 0 };

    return ::ClearCommError(m_hComm, &dwError, &comStat);
}

void CSerialPort::Close()
{
    m_bExit = true;
    if (nullptr != m_hReadEvent)
    {
        ::SetEvent(m_hReadEvent);
        ::CloseHandle(m_hReadEvent);
        m_hReadEvent = nullptr;
    }

    if (nullptr != m_hWriteEvent)
    {
        ::SetEvent(m_hWriteEvent);
        ::CloseHandle(m_hWriteEvent);
        m_hWriteEvent = nullptr;
    }

    if (INVALID_HANDLE_VALUE != m_hComm)
    {
        ::CancelIoEx(m_hComm, nullptr);
        ::CloseHandle(m_hComm);
        m_hComm = INVALID_HANDLE_VALUE;
    }
}

bool CSerialPort::OpenComm(int nComm)
{
    TCHAR szBuf[MAX_PATH];
    (void)::StringCchPrintf(szBuf, _countof(szBuf), _T(R"(\\.\COM%d)"), nComm);

    this->Close();

    m_hComm = ::CreateFile(
            szBuf,
            GENERIC_READ | GENERIC_WRITE,   
            0,                             
            nullptr,
            OPEN_EXISTING,                  
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,          
            nullptr
    );

    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    m_hReadEvent = CreateEvent(nullptr, false, false, nullptr);
    if (nullptr == m_hReadEvent)
    {
        this->Close();
        return false;
    }

    m_hWriteEvent = CreateEvent(nullptr, false, false, nullptr);
    if (nullptr == m_hWriteEvent)
    {
        this->Close();
        return false;
    }

    return true;
}

bool CSerialPort::OpenComm(const _tstring& strName)
{
    TCHAR szBuf[MAX_PATH];
    (void)::StringCchPrintf(szBuf, _countof(szBuf), _T(R"(\\.\%s)"), strName.c_str());

    this->Close();

    m_hComm = ::CreateFile(
            szBuf,
            GENERIC_READ | GENERIC_WRITE,   
            0,                              
            nullptr,
            OPEN_EXISTING,                  
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,           
            nullptr
    );

    if (INVALID_HANDLE_VALUE == m_hComm)
    {
        return false;
    }

    m_hReadEvent = CreateEvent(nullptr, false, false, nullptr);
    if (nullptr == m_hReadEvent)
    {
        this->Close();
        return false;
    }

    m_hWriteEvent = CreateEvent(nullptr, false, false, nullptr);
    if (nullptr == m_hWriteEvent)
    {
        this->Close();
        return false;
    }

    return true;
}

bool CSerialPort::WaitForOverlapped(
        HANDLE hFile,
        LPOVERLAPPED lpOv,
        LPDWORD lpBytesTransferred,
        DWORD nTimeOut
)
{
    bool bResult = false;
    DWORD dwBytesTransferred = 0;
    DWORD dwWait = ::WaitForSingleObject(lpOv->hEvent, nTimeOut);

    switch (dwWait)
    {
        case WAIT_OBJECT_0:
        {
            if (::GetOverlappedResult(hFile,
                                      lpOv,
                                      &dwBytesTransferred,
                                      true
            ))
            {
                bResult = true;
            }
            else
            {
                bResult = false;
            }

            if (lpBytesTransferred)
            {
                *lpBytesTransferred = dwBytesTransferred;
            }
        }
            break;
        case WAIT_TIMEOUT:
            break;
        default:
            break;
    }

    if (!bResult && !m_bExit)
    {
        ::CancelIoEx(hFile, lpOv);
    }

    return bResult;
}

bool CSerialPort::Write(
        LPCVOID lpData,
        DWORD cbSize,
        LPDWORD lpBytesWritten,
        DWORD nTimeOut/* = 1000*/
)
{
    if (INVALID_HANDLE_VALUE == m_hComm ||
        nullptr == lpData ||
        0 == cbSize ||
        m_bExit)
    {
        return false;
    }

    OVERLAPPED ov = { 0 };
    BOOL bResult = FALSE;
    ov.hEvent = m_hWriteEvent;

    do
    {
        bResult = ::WriteFile(
                m_hComm,
                lpData,
                cbSize,
                lpBytesWritten,
                &ov
        );

        if (bResult)
        {
            break;
        }

        if (ERROR_IO_PENDING != ::GetLastError())
        {
            break;
        }

        bResult = WaitForOverlapped(m_hComm, &ov, lpBytesWritten, nTimeOut);

    } while (false);

    return bResult;
}

bool CSerialPort::Read(
        LPVOID lpData,
        LPDWORD lpBytesRead/* = nullptr*/,
        DWORD nTimeOut/* = 3000*/
)
{
    DWORD errors;               
    DWORD read_len;             
    COMSTAT comstat;            

    if (INVALID_HANDLE_VALUE == m_hComm ||
        nullptr == lpData ||
        m_bExit)
    {
        return false;
    }

    if (!ClearCommError(m_hComm, &errors, &comstat)) {
        return FALSE;
    }

    read_len = comstat.cbInQue;

    OVERLAPPED ov = { 0 };
    BOOL bResult = FALSE;
    ov.hEvent = m_hReadEvent;

    do
    {
        bResult = ::ReadFile(
                m_hComm,
                lpData,
                read_len,
                lpBytesRead,
                &ov
        );

        if (bResult)
        {
            break;
        }

        if (ERROR_IO_PENDING != ::GetLastError())
        {
            break;
        }

        bResult = WaitForOverlapped(m_hComm, &ov, lpBytesRead, nTimeOut);

    } while (false);

    return bResult;
}