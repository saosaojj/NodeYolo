# Modbus TCP 模拟器，用于测试环境中模拟PLC设备，提供线圈和寄存器的读写服务
import argparse

from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock


# 主函数：解析命令行参数，初始化Modbus数据存储并启动TCP服务器
def main():
    parser = argparse.ArgumentParser(description='ModbusTCP Simulator for testing')
    parser.add_argument('--port', type=int, default=5020, help='Port to listen on')
    parser.add_argument('--address', type=str, default='localhost', help='Address to bind to')
    args = parser.parse_args()

    # 初始化各数据块：线圈、离散输入、保持寄存器、输入寄存器，各256个地址
    coil_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    discrete_input_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    holding_register_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    input_register_block = ModbusSequentialDataBlock(0x00, [0] * 256)

    # 创建从站上下文，包含四种数据块
    slave_context = ModbusSlaveContext(
        co=coil_block,
        di=discrete_input_block,
        hr=holding_register_block,
        ir=input_register_block,
    )

    # 创建服务器上下文，单从站模式
    server_context = ModbusServerContext(slaves=slave_context, single=True)

    print(f'Starting ModbusTCP simulator on {args.address}:{args.port}')

    # 启动Modbus TCP服务器，阻塞运行
    StartTcpServer(
        context=server_context,
        address=(args.address, args.port),
    )


if __name__ == '__main__':
    main()
