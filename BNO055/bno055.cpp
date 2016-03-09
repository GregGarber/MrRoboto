#include "bno055.h"

BNO055::BNO055(QObject *parent) : QObject(parent)
{
    // For now make stupid assumption that 1st available serial port is correct one
    qDebug() << "Serial ports info:";
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        qDebug() << QObject::tr("Port: ") + info.portName() + "\n"
                    + QObject::tr("Location: ") + info.systemLocation() + "\n"
                    + QObject::tr("Description: ") + info.description() + "\n"
                    + QObject::tr("Manufacturer: ") + info.manufacturer() + "\n"
                    + QObject::tr("Serial number: ") + info.serialNumber() + "\n"
                    + QObject::tr("Vendor Identifier: ") + (info.hasVendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : QString()) + "\n"
                    + QObject::tr("Product Identifier: ") + (info.hasProductIdentifier() ? QString::number(info.productIdentifier(), 16) : QString()) + "\n"
                    + QObject::tr("Busy: ") + (info.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) + "\n";
        if(!info.isBusy()){
            serial_port.setPortName(info.systemLocation());
            //serial_port.setPortName("/dev/ttyAMA0");
            serial_port.setBaudRate(QSerialPort::Baud115200);
            serial_port.setDataBits(QSerialPort::Data8);
            serial_port.setParity(QSerialPort::NoParity);
            serial_port.setFlowControl(QSerialPort::NoFlowControl);
            serial_port.open(QIODevice::ReadWrite);
            if(serial_port.isOpen()){
                qDebug() << "Opened Serial Port: " << info.systemLocation();
                break;
            }else{
                qDebug() << "Couldn't Open Serial Port: " << info.systemLocation();
            }
        }
    }
    if(!serial_port.isOpen()){
            throw std::runtime_error(std::string("Couldn't open a serial port!"));
    }
}

void BNO055::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        qDebug() << "Critical ResourceError:" << serial_port.errorString();
    }else{
        qDebug() << "Error:"<< error << " " << serial_port.errorString();
    }
}


bool BNO055::writeCommand(quint8 address, QByteArray value, bool ack=true)
{
    QByteArray send_data, response;
    int cnt = 0;
    send_data.append(0xAA);
    send_data.append( (char) 0x00);
    send_data.append( address );
    send_data.append( value.count() );
    send_data.append( value );
    do{
        serial_port.write(send_data, send_data.count());
        response = serial_port.readAll();
        cnt++;
    }while( ack && (cnt < 5) && !isWriteAck(response));
    return ack ? isWriteAck(response) : true;
}

bool BNO055::isWriteAck(QByteArray response){
    //4.7.1 Register write: Write ACK 0xEE +
    if((uchar)response[0] == (uchar)0xEE){
        switch(response[1]){
        case WRITE_SUCCESS:
            qDebug() << "write_ack() WRITE_SUCCESS";
            return true;
            break;
        case WRITE_FAIL:
            qDebug() << "write_ack() WRITE_FAIL";
            break;
        case REGMAP_INVALID_ADDRESS:
            qDebug() << "write_ack() REGMAP_INVALID_ADDRESS";
            break;
        case REGMAP_WRITE_DISABLED:
            qDebug() << "write_ack() REGMAP_WRITE_DISABLED";
            break;
        case WRONG_START_BYTE:
            qDebug() << "write_ack() WRONG_START_BYTE";
            break;
        case BUS_OVER_RUN_ERROR:
            qDebug() << "write_ack() BUS_OVER_RUN_ERROR";
            break;
        case MAX_LENGTH_ERROR:
            qDebug() << "write_ack() MAX_LENGTH_ERROR";
            break;
        case MIN_LENGTH_ERROR:
            qDebug() << "write_ack() MIN_LENGTH_ERROR";
            break;
        case RECEIVE_CHARACTER_TIMEOUT:
            qDebug() << "write_ack() RECEIVE_CHARACTER_TIMEOUT";
            break;
        default:
            qDebug() << "write_ack() Unknown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
            break;
        }
    }else if((uchar)response[0] == 0xBB){
        qDebug() << "write success";
    }else{
        qDebug() << "write_ack() Unknown Header:"<< QString("%1").arg((uchar)response[0],1,16)
                        <<"Unknown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
    }
    return false;
}


bool BNO055::readCommand(quint8 address, QByteArray value, QByteArray &response, bool ack=true)
{
    QByteArray send_data;
    bool acknowledged;
    int cnt = 0;
    send_data.append(0xAA);
    send_data.append( (char)0x00);
    send_data.append( address );
    send_data.append( value.count() );
    send_data.append( value );
    do{
        serial_port.write(send_data, send_data.count());
        response = serial_port.readAll();
        cnt++;
        acknowledged = isReadAck(response);
    }while( ack && (cnt < 5) && !acknowledged);
    return ack ? acknowledged : ( cnt < 5 );
}

bool BNO055::isReadAck(QByteArray response){
    // 4.7.2 Register read: Read Failure or Read ACK 0xEE+
    if((uchar)response[0] == (uchar)0xEE){
        switch(response[1]){
        case READ_SUCCESS:
            // READ_SUCCESS is not specified in manual so guessed and may never get here
            qDebug() << "read_ack() READ_SUCCESS";
            return true;
            break;
        case READ_FAIL:
            qDebug() << "read_ack() Error: READ_FAIL";
            break;
        case REGMAP_INVALID_ADDRESS:
            qDebug() << "read_ack() Error:  REGMAP_INVALID_ADDRESS";
            break;
        case REGMAP_WRITE_DISABLED:
            qDebug() << "read_ack() Error:  REGMAP_WRITE_DISABLED";
            break;
        case WRONG_START_BYTE:
            qDebug() << "read_ack() Error:  WRONG_START_BYTE";
            break;
        case BUS_OVER_RUN_ERROR:
            qDebug() << "read_ack() Error:  BUS_OVER_RUN_ERROR";
            break;
        case MAX_LENGTH_ERROR:
            qDebug() << "read_ack() Error:  MAX_LENGTH_ERROR";
            break;
        case MIN_LENGTH_ERROR:
            qDebug() << "read_ack() Error:  MIN_LENGTH_ERROR";
            break;
        case RECEIVE_CHARACTER_TIMEOUT:
            qDebug() << "read_ack() Error:  RECEIVE_CHARACTER_TIMEOUT";
            break;
        default:
            qDebug() << "read_ack() Unkown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
            break;
        }
    }else if((uchar)response[0] == (uchar)0xBB){
        qDebug() << "read success";
        return true;
    }else{
        qDebug() << "read_ack() Unknown Header:"<< QString("%1").arg((uchar)response[0],1,16)
                        <<"Unknown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
    }
    return false;
}

bool BNO055::setMode(char mode){
    QByteArray m;// well this is awkward
    m.append(mode);
    writeCommand(BNO055_OPR_MODE_ADDR, m, true);
}
bool BNO055::setConfigMode(){
    setMode(OPERATION_MODE_CONFIG);
}
/*
    def _write_byte(self, address, value, ack=True):
        if self._i2c_device is not None:
            # I2C write.
            self._i2c_device.write8(address, value)
        else:
            # Build and send serial register write command.
            command = bytearray(5)
            command[0] = 0xAA  # Start byte
            command[1] = 0x00  # Write
            command[2] = address & 0xFF
            command[3] = 1     # Length (1 byte)
            command[4] = value & 0xFF
            resp = self._serial_send(command, ack=ack)
            # Verify register write succeeded if there was an acknowledgement.
            if ack and resp[0] != 0xEE and resp[1] != 0x01:
                raise RuntimeError('Register write error: 0x{0}'.format(binascii.hexlify(resp)))
      */
