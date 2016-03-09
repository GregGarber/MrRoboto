#include "bno055.h"

BNO055::BNO055(QObject *parent) : QObject(parent)
{
    wiringPiSetup () ;
    pinMode (18, OUTPUT) ;
    digitalWrite (0, HIGH) ; delay (650) ;


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
            serial_port.clear();
            //blocking
            serial_port.waitForReadyRead(1000);
            serial_port.waitForBytesWritten(1000);
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
    connect(&serial_port, SIGNAL(error(QSerialPort::SerialPortError)), this,
            SLOT(handleError(QSerialPort::SerialPortError)));
}

void BNO055::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        qDebug() << "Critical ResourceError:" << serial_port.errorString();
    }/*else{
        qDebug() << "Error:"<< error << " " << serial_port.errorString();
    }*/
}

QByteArray BNO055::serialSend(QByteArray command, bool ack){
    QByteArray acknowledgement, response, tmp;
    int cnt = 0;
    bool is_ack = false;
    qDebug() << "sending:" << command.toHex();
    serial_port.clear();
    do{
        serial_port.write(command, command.count());
        if(!ack) return response;
        cnt++;
        do{
            tmp = serial_port.read(2 - acknowledgement.count() );//attempt limit read to 2 bytes
            acknowledgement.append(tmp);
        }while( acknowledgement.count() < 2 && serial_port.waitForReadyRead(1000));

        if(acknowledgement.count() != 2) {
            throw std::runtime_error(std::string("Didn't Get Acknowledgement!"));
        }
    //    qDebug()<< "ack:" << acknowledgement.toHex();
        is_ack =  (quint8) command[1] == (quint8) 0x00 ? isWriteAck(acknowledgement) : isReadAck(acknowledgement); 
        if(!is_ack) continue;
        int length_to_read = acknowledgement[1];
        tmp.clear();
        do{
            tmp = serial_port.read(length_to_read - response.count() );//attempt limit read to length_to_read bytes
            response.append(tmp);
        }while( response.count() < length_to_read && serial_port.waitForReadyRead(1000));
    }while( !is_ack &&  cnt < 5 );
    qDebug()<< "response:" << response.toHex();
    return response;
}

bool BNO055::writeCommand(quint8 address, QByteArray value, bool ack)
{
    QByteArray send_data;
    send_data.append(0xAA);
    send_data.append( (char) 0x00);
    send_data.append( address );
    send_data.append( value.count() );
    send_data.append( value );
    serialSend( send_data, ack );
    return true;
}

bool BNO055::writeCommand(quint8 address, quint8 value, bool ack)
{
    QByteArray v;
    v.append(value);
    return writeCommand(address, v, ack);
}

bool BNO055::isWriteAck(QByteArray response){
    //4.7.1 Register write: Write ACK 0xEE +
    if((uchar)response[0] == (uchar)0xEE){
        switch(response[1]){
            case WRITE_SUCCESS:
//                qDebug() << "isWriteAck() WRITE_SUCCESS";
                return true;
                break;
            case WRITE_FAIL:
                qDebug() << "isWriteAck() WRITE_FAIL";
                return false;
                break;
            case REGMAP_INVALID_ADDRESS:
                qDebug() << "isWriteAck() REGMAP_INVALID_ADDRESS";
                return false;
                break;
            case REGMAP_WRITE_DISABLED:
                qDebug() << "isWriteAck() REGMAP_WRITE_DISABLED";
                return false;
                break;
            case WRONG_START_BYTE:
                qDebug() << "isWriteAck() WRONG_START_BYTE";
                return false;
                break;
            case BUS_OVER_RUN_ERROR:
                qDebug() << "isWriteAck() BUS_OVER_RUN_ERROR";
                return false;
                break;
            case MAX_LENGTH_ERROR:
                qDebug() << "isWriteAck() MAX_LENGTH_ERROR";
                return false;
                break;
            case MIN_LENGTH_ERROR:
                qDebug() << "isWriteAck() MIN_LENGTH_ERROR";
                return false;
                break;
            case RECEIVE_CHARACTER_TIMEOUT:
                qDebug() << "isWriteAck() RECEIVE_CHARACTER_TIMEOUT";
                return false;
                break;
            default:
                qDebug() << "isWriteAck() Unknown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
                break;
        }
    }
    return true;
}


QByteArray BNO055::readCommand(quint8 address, QByteArray value, bool ack)
{
    QByteArray send_data;
    send_data.append(0xAA);
    send_data.append( (char)0x01);
    send_data.append( address );
    send_data.append( value.count() );
    send_data.append( value );
    return serialSend( send_data, ack );
}

QByteArray BNO055::readCommand(quint8 address, quint8 value, bool ack)
{
    QByteArray v;
    v.append(value);
    return readCommand(address, v, ack);
}

QByteArray BNO055::readBytes(quint8 address, quint8 length )
{
    QByteArray send_data;
    send_data.append(0xAA);
    send_data.append( (char)0x01);
    send_data.append( address );
    send_data.append( length );
    return serialSend( send_data, true );
}

quint8 BNO055::readByte(quint8 address )
{
    return readBytes(address, 1)[0];
}

bool BNO055::isReadAck(QByteArray response){
    // 4.7.2 Register read: Read Failure or Read ACK 0xEE+
    if((uchar)response[0] == (uchar)0xEE){
        switch(response[1]){
        case READ_SUCCESS:
            // READ_SUCCESS is not specified in manual so guessed and may never get here
            qDebug() << "isReadAck() READ_SUCCESS";
            return true;
            break;
        case READ_FAIL:
            qDebug() << "isReadAck() Error: READ_FAIL";
            return false;
            break;
        case REGMAP_INVALID_ADDRESS:
            qDebug() << "isReadAck() Error:  REGMAP_INVALID_ADDRESS";
            return false;
            break;
        case REGMAP_WRITE_DISABLED:
            qDebug() << "isReadAck() Error:  REGMAP_WRITE_DISABLED";
            return false;
            break;
        case WRONG_START_BYTE:
            qDebug() << "isReadAck() Error:  WRONG_START_BYTE";
            return false;
            break;
        case BUS_OVER_RUN_ERROR:
            qDebug() << "isReadAck() Error:  BUS_OVER_RUN_ERROR";
            return false;
            break;
        case MAX_LENGTH_ERROR:
            qDebug() << "isReadAck() Error:  MAX_LENGTH_ERROR";
            return false;
            break;
        case MIN_LENGTH_ERROR:
            qDebug() << "isReadAck() Error:  MIN_LENGTH_ERROR";
            return false;
            break;
        case RECEIVE_CHARACTER_TIMEOUT:
            qDebug() << "isReadAck() Error:  RECEIVE_CHARACTER_TIMEOUT";
            return false;
            break;
        default:
            qDebug() << "isReadAck() Unkown Response Code:"<<QString("%1").arg((uchar)response[1],1,16);
            break;
        }
    }
    return true;
}

bool BNO055::setMode(char mode){
    bool r = writeCommand(BNO055_OPR_MODE_ADDR, mode, true);
    delay (300) ;
    return r;
}

bool BNO055::setConfigMode(){
    return setMode(OPERATION_MODE_CONFIG);
}

bool BNO055::setOperationMode(){
    return setMode(OPERATION_MODE_NDOF);
}


void BNO055::init(){
    QByteArray response;
    qDebug() << "write PAGE ID";
    writeCommand(BNO055_PAGE_ID_ADDR, 0, false);
    qDebug() << "Config Mode";
    setConfigMode();
    qDebug() << "write PAGE ID";
    writeCommand(BNO055_PAGE_ID_ADDR, 0);
    qDebug() << "read CHIP ID";
    response=readCommand(BNO055_CHIP_ID_ADDR, 1);
    qDebug() << "CHIP ID:" << response.toHex();
    if( (quint8) response[0] !=  (quint8) BNO055_ID){
            throw std::runtime_error(std::string("Doesn't appear to be a BNO055"));
    }

    //reset
    digitalWrite (0, LOW) ; delay (10) ;
    digitalWrite (0, HIGH) ; 
    delay (650) ;

    qDebug() << "write Power mode";
    writeCommand(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    qDebug() << "write use internal oscillator";
    writeCommand(BNO055_SYS_TRIGGER_ADDR, 0x0);
    setOperationMode();

    //probably don't want this here forever
    Revision rev = getRevision();
    qDebug() << "Revision Information";
    qDebug() << "Accelerometer" << rev.accel;
    qDebug() << "Magnetometer" << rev.mag;
    qDebug() << "Gyroscope" << rev.gyro;
    qDebug() << "bl" << rev.bl;
    qDebug() << "sw" << rev.sw;
    //probably don't want this here forever
    for(int i = 0; i< 10; i++){
        qreal* euler = readEuler();
        qDebug() << (qreal)euler[0] << " " << (qreal)euler[1] << " " << (qreal)euler[2];
    } 
}

quint16 BNO055::bytes2quint16(quint8 lsb, quint8 msb){
    return ((msb << 8) | lsb) & 0xFFFF;
}

Revision BNO055::getRevision(){
    Revision rev;
    delay (3000) ;
    rev.accel = readByte(BNO055_ACCEL_REV_ID_ADDR);
    rev.mag = readByte(BNO055_MAG_REV_ID_ADDR);
    rev.gyro = readByte(BNO055_GYRO_REV_ID_ADDR);
    rev.bl = readByte(BNO055_BL_REV_ID_ADDR);
    rev.sw = bytes2quint16( readByte(BNO055_SW_REV_ID_LSB_ADDR), readByte(BNO055_SW_REV_ID_MSB_ADDR) );
    return rev;
}



qint16* BNO055::readVector3(quint8 address){
    QByteArray data = readBytes(address, 6);
    qint16 *result = new qint16[3];
    for(int i=0; i<3; i++){
        result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF;
        //if(result[i] > 32767) result[i] -= 65536;
    }
    return result;
}

qreal* BNO055::readEuler(){
    qint16 *data = readVector3(BNO055_EULER_H_LSB_ADDR);
    qreal *result = new qreal[3];
    result[0] = (qreal)data[0] / 16.0;
    result[1] = (qreal)data[1] / 16.0;
    result[2] = (qreal)data[2] / 16.0;
    return result;
}
