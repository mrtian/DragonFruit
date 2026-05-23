# DragonFruit
## 这是我学习和测试火龙果一号的代码地方
## 步骤
### 1.烧录轮毂电机驱动板
进行 MM32SPIN05_BOARD_BLDC_DRIVER 执行 `make`(文件夹中已经有生成的 elf/hex/,可直接烧录),执行通过后会生成 fireware.hex,将 fireware.hex 烧录进主板，烧录命令（需要安装 pyocd）：
> pyocd flash -t mm32spin05pf firmware.hex   
**注：新主板需要先清除主板芯片的原有固件，不然烧不进去：**   
>
>> pyocd erase -t mm32spin05pf --chip  

**注：副板，请将 MM32SPIN05_BOARD_BLDC_DRIVER 文件夹中 `src/config.h` 的`IS_MASTER_BORAD` 改为 0 再 `make`和烧录！！！！！！！**  

### 2，接线调试
将上位机的uart 接入驱动板的 uart 接口（只需接 3 根线,RX,TX,GND,12V接到副板 UART 插座的 12V），轮毂电机按 UVW 与霍尔接好，将开关（**请使用自复位开关**）接入开关插座（主板红色的），这样主副板就可以跟主板同步开关机了。
