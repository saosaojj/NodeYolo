import argparse

from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock


def main():
    parser = argparse.ArgumentParser(description='ModbusTCP Simulator for testing')
    parser.add_argument('--port', type=int, default=5020, help='Port to listen on')
    parser.add_argument('--address', type=str, default='localhost', help='Address to bind to')
    args = parser.parse_args()

    coil_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    discrete_input_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    holding_register_block = ModbusSequentialDataBlock(0x00, [0] * 256)
    input_register_block = ModbusSequentialDataBlock(0x00, [0] * 256)

    slave_context = ModbusSlaveContext(
        co=coil_block,
        di=discrete_input_block,
        hr=holding_register_block,
        ir=input_register_block,
    )

    server_context = ModbusServerContext(slaves=slave_context, single=True)

    print(f'Starting ModbusTCP simulator on {args.address}:{args.port}')

    StartTcpServer(
        context=server_context,
        address=(args.address, args.port),
    )


if __name__ == '__main__':
    main()
