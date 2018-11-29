import smbus
bus = smbus.SMBus(1)
from LSM9DS0 import *
from LSM9DS1 import *
LSM9DS0 = 1
def detectIMU():
    global LSM9DS0
    try:
        LSM9DS0_WHO_G_response = (bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_WHO_AM_I_G))
        LSM9DS0_WHO_XM_response = (bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_WHO_AM_I_XM))
    except IOError as e:
        print('')
    else:
        if (LSM9DS0_WHO_G_response == 0xd4) and (LSM9DS0_WHO_XM_response == 0x49):
            LSM9DS0 = 1
    try:
        LSM9DS1_WHO_XG_response = (bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_WHO_AM_I_XG))
        LSM9DS1_WHO_M_response = (bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_WHO_AM_I_M))
    except IOError as f:
        print('')
    else:
        if (LSM9DS1_WHO_XG_response == 0x68) and (LSM9DS1_WHO_M_response == 0x3d):
            LSM9DS0 = 0
def writeAG(register,value):
    bus.write_byte_data(ACC_ADDRESS , register, value)
    return -1
def writeACC(register,value):
    bus.write_byte_data(ACC_ADDRESS , register, value)
    return -1
def writeMAG(register,value):
    bus.write_byte_data(MAG_ADDRESS, register, value)
    return -1
def writeGRY(register,value):
    bus.write_byte_data(GYR_ADDRESS, register, value)
    return -1
def readACCx():
    acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_XL)
    acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readACCy():
    acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_XL)
    acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readACCz():
    acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_XL)
    acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readMAGx():
    mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
    mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readMAGy():
    mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
    mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readMAGz():
    mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
    mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readGYRx():
    gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
    gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def readGYRy():
    gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
    gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def readGYRz():
    gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
    gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def writeACC(register,value):
    if(LSM9DS0):
        bus.write_byte_data(LSM9DS0_ACC_ADDRESS , register, value)
    else:
        bus.write_byte_data(LSM9DS1_ACC_ADDRESS , register, value)
    return -1
def writeMAG(register,value):
    if(LSM9DS0):
        bus.write_byte_data(LSM9DS0_MAG_ADDRESS, register, value)
    else:
        bus.write_byte_data(LSM9DS1_MAG_ADDRESS , register, value)        
        return -1
def writeGRY(register,value):
    if(LSM9DS0):
        bus.write_byte_data(LSM9DS0_GYR_ADDRESS, register, value)
    else:
        bus.write_byte_data(LSM9DS1_GYR_ADDRESS , register, value)        
        return -1
def readACCx():
    if (LSM9DS0):
        acc_l = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_X_L_A)
        acc_h = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_X_H_A)
    else:
        acc_l = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_X_L_XL)
        acc_h = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_X_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readACCy():
    if (LSM9DS0):
        acc_l = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_Y_L_A)
        acc_h = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_Y_H_A)
    else:
        acc_l = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_Y_L_XL)
        acc_h = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_Y_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readACCz():
    if (LSM9DS0):
        acc_l = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_Z_L_A)
        acc_h = bus.read_byte_data(LSM9DS0_ACC_ADDRESS, LSM9DS0_OUT_Z_H_A)
    else:
        acc_l = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_Z_L_XL)
        acc_h = bus.read_byte_data(LSM9DS1_ACC_ADDRESS, LSM9DS1_OUT_Z_H_XL)
    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536
def readMAGx():
    if (LSM9DS0):
        mag_l = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_X_L_M)
        mag_h = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_X_H_M)
    else:
        mag_l = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_X_L_M)
        mag_h = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_X_H_M)
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readMAGy():
    if (LSM9DS0):
        mag_l = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_Y_L_M)
        mag_h = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_Y_H_M)
    else:
        mag_l = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_Y_L_M)
        mag_h = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_Y_H_M)
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readMAGz():
    if (LSM9DS0):
        mag_l = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_Z_L_M)
        mag_h = bus.read_byte_data(LSM9DS0_MAG_ADDRESS, LSM9DS0_OUT_Z_H_M)
    else:
        mag_l = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_Z_L_M)
        mag_h = bus.read_byte_data(LSM9DS1_MAG_ADDRESS, LSM9DS1_OUT_Z_H_M)           
    mag_combined = (mag_l | mag_h <<8)
    return mag_combined  if mag_combined < 32768 else mag_combined - 65536
def readGYRx():
    if (LSM9DS0):
        gyr_l = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_X_L_G)
        gyr_h = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_X_H_G)
    else:
        gyr_l = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_X_L_G)
        gyr_h = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_X_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def readGYRy():
    if (LSM9DS0):
        gyr_l = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_Y_L_G)
        gyr_h = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_Y_H_G)
    else:
        gyr_l = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_Y_L_G)
        gyr_h = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_Y_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def readGYRz():
    if (LSM9DS0):
        gyr_l = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_Z_L_G)
        gyr_h = bus.read_byte_data(LSM9DS0_GYR_ADDRESS, LSM9DS0_OUT_Z_H_G)
    else:
        gyr_l = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_Z_L_G)
        gyr_h = bus.read_byte_data(LSM9DS1_GYR_ADDRESS, LSM9DS1_OUT_Z_H_G)
    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
def initIMU():
    if (LSM9DS0):
        writeACC(LSM9DS0_CTRL_REG1_XM, 0b01100111)
        writeACC(LSM9DS0_CTRL_REG2_XM, 0b00100000)
        writeMAG(LSM9DS0_CTRL_REG5_XM, 0b11110000)
        writeMAG(LSM9DS0_CTRL_REG6_XM, 0b01100000)
        writeMAG(LSM9DS0_CTRL_REG7_XM, 0b00000000)
        writeGRY(LSM9DS0_CTRL_REG1_G, 0b00001111)
        writeGRY(LSM9DS0_CTRL_REG4_G, 0b00110000)
    else:
        writeGRY(LSM9DS1_CTRL_REG4,0b00111000)
        writeGRY(LSM9DS1_CTRL_REG1_G,0b10111000)
        writeGRY(LSM9DS1_ORIENT_CFG_G,0b00111000)
        writeACC(LSM9DS1_CTRL_REG5_XL,0b00111000)
        writeACC(LSM9DS1_CTRL_REG6_XL,0b00101000)
        writeMAG(LSM9DS1_CTRL_REG1_M, 0b10011100)
        writeMAG(LSM9DS1_CTRL_REG2_M, 0b01000000)
        writeMAG(LSM9DS1_CTRL_REG3_M, 0b00000000)
        writeMAG(LSM9DS1_CTRL_REG4_M, 0b00000000)
