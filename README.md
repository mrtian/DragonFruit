# DragonFruit
## 这是我学习和测试火龙果一号的代码地方
## 步骤
### 1.烧录轮毂电机驱动板
进行MM32SPIN05_BOARD_BLDC_DRIVER 执行 make,执行通过后会生成 fireware.elf,将 fireware.elf 烧录进主板，烧录命令（需要安装 pyocd）：
> pyocd flash -t mm32spin05pf firmware.hex
注：新主板需要先清除主板芯片自还的固件，不然烧不进去：
> pyocd erase -t mm32spin05pf --chip