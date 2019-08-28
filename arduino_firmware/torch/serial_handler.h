#ifndef SERIAL_HANDLER_H_
#define SERIAL_HANDLER_H_

class SerialHandler
{
    public:
    SerialHandler();
    SerialHandler(char*, char*);

    public:
    void Receive();
    bool IsSerialActive();
    bool IsDataReady();
    void DeactivateSerial();
    void SetDataState(bool state);

    private:
    char *inbound_buffer_;
    char *outbound_buffer_;
    char char_buf_;
    bool serial_active_;
    bool data_ready_;
    int inbound_cursor_;
    int outbound_cursor_;
};

SerialHandler::SerialHandler()
  : serial_active_(false), data_ready_(false),
    inbound_buffer_(NULL), outbound_buffer_(NULL)
{}

SerialHandler::SerialHandler(char *inbound_buf, char *outbound_buf)
  : serial_active_(false), data_ready_(false),
    inbound_buffer_(inbound_buf), outbound_buffer_(outbound_buf)
{}

void SerialHandler::Receive()
{
    if (Serial.available())
    {
        if (!serial_active_)
        {
            serial_active_ = true;
            data_ready_ = false;
            inbound_cursor_ = -1;
        }

        char_buf_ = Serial.read();
        inbound_buffer_[++inbound_cursor_] = char_buf_;

        if (char_buf_ == '\n')
        {
            inbound_buffer_[inbound_cursor_] == '\0';
            data_ready_ = true;
            inbound_cursor_ = -1;
        }
    }
}

bool SerialHandler::IsSerialActive()
{
    return serial_active_;
}

bool SerialHandler::IsDataReady()
{
    return data_ready_;
}

void SerialHandler::DeactivateSerial()
{
    serial_active_ = false;
}

void SerialHandler::SetDataState(bool state)
{
    data_ready_ = state;
}

#endif
