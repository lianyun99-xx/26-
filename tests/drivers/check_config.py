"""Check generated code against CubeMX configuration without regenerating it."""
import re
from pathlib import Path
root = Path(__file__).resolve().parents[2]
ioc = dict(line.split('=', 1) for line in (root / '27 dog1.ioc').read_text(encoding='utf-8-sig').splitlines() if '=' in line)
code = (root / 'Core/Src/fdcan.c').read_text()
intervals = []
for bus, offset in enumerate((0, 20, 40), 1):
    fields = dict(re.findall(rf'hfdcan{bus}\.Init\.(\w+) = ([\w]+);', code))
    assert int(fields['MessageRAMOffset']) == offset
    assert int(ioc.get(f'FDCAN{bus}.MessageRAMOffset', '0')) == offset
    for key in ('StdFiltersNbr', 'ExtFiltersNbr', 'RxFifo0ElmtsNbr', 'RxFifo1ElmtsNbr', 'RxBuffersNbr', 'TxEventsNbr', 'TxBuffersNbr', 'TxFifoQueueElmtsNbr'):
        assert int(ioc.get(f'FDCAN{bus}.{key}', '0')) == int(fields[key]), (bus, key)
    # Standard filter: 1 word; classic 8-byte RX/TX elements: 4 words each.
    words = int(fields['StdFiltersNbr']) + 2 * int(fields['ExtFiltersNbr'])
    for key in ('RxFifo0ElmtsNbr', 'RxFifo1ElmtsNbr', 'RxBuffersNbr', 'TxBuffersNbr', 'TxFifoQueueElmtsNbr'):
        words += 4 * int(fields[key])
    words += 2 * int(fields['TxEventsNbr'])
    assert words == 17
    intervals.append((offset, offset + words))
assert all(a[1] <= b[0] for a, b in zip(intervals, intervals[1:]))
tasks = {t.split(',')[0]: int(t.split(',')[2]) for t in ioc['FREERTOS.Tasks01'].split(';')}
for name, words in (('BMI088', 256), ('AttitudeControl', 512), ('Moror3508', 256)):
    assert tasks[name] == words
    freertos = (root / 'Core/Src/freertos.c').read_text()
    assert re.search(rf'\.name = "{name}",\s*\.stack_size = {words} \* 4', freertos)
for pin in ('PE10', 'PE12'):
    assert ioc[f'{pin}.Signal'] == 'GPIO_Input'
rtos_config = (root / 'Core/Inc/FreeRTOSConfig.h').read_text()
assert re.search(r'#define configTICK_RATE_HZ\s+\(\(TickType_t\)1000\)', rtos_config)
main = (root / 'Core/Src/main.c').read_text()
freertos = (root / 'Core/Src/freertos.c').read_text()
assert main.count('BMI088_init();') == 1
assert main.index('MX_SPI2_Init();') < main.index('BMI088_init();') < main.index('MX_FREERTOS_Init();')
assert 'BMI088_init();' not in freertos and 'Attitude_Init();' not in freertos
assert '(void)BMI088Read();' in freertos and 'Attitude_Update(' not in freertos
assert 'Motor3508_SendCommand(&hfdcan1)' in freertos and 'Motor3508_SendCommand(&hfdcan2)' in freertos
print(f'PASS configuration: RAM {intervals}, task stacks, polling GPIO')
print('PASS startup init order, unified BMI088Read and per-bus Motor3508 Send task integration')
