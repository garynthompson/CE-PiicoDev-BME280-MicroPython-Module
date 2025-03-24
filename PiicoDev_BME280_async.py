# An asyncio version of PiicoDev_BME280
# noinspection PyProtectedMember
from PiicoDev_Unified import _SYSNAME, i2c_err_str

from PiicoDev_BME280 import PiicoDev_BME280

if _SYSNAME == 'Linux':
    from asyncio import sleep as a_sleep_s

    async def a_sleep_ms(t: int):
        await a_sleep_s(t/1000)

else:
    # noinspection PyUnresolvedReferences
    from asyncio import sleep_ms as a_sleep_ms


# noinspection DuplicatedCode,PyPep8Naming,SpellCheckingInspection
class A_PiicoDev_BME280(PiicoDev_BME280):
    """
    Caution:  The constructor is blocking and implements sleep_ms.

    I2C Bus reads are not asynchronous, however, sleeps while waiting for
    the chip to settle are.
    """

    async def read_raw_data(self):
        self._write8(0xF4, (self.p_mode << 5 | self.t_mode << 2 | 1))
        sleep_time = 1250
        if self.t_mode in [1, 2, 3, 4, 5]:
            sleep_time += 2300*(1<< self.t_mode)
        if self.p_mode in [1, 2, 3, 4, 5]:
            sleep_time += 575+(2300*(1<<self.p_mode))
        if self.h_mode in [1, 2, 3, 4, 5]:
            sleep_time += 575+(2300*(1<<self.h_mode))
        await a_sleep_ms(1+sleep_time//1000)
        while self._read16(0xF3) & 0x08:
            await a_sleep_ms(1)
        raw_p = ((self._read8(0xF7)<<16)|(self._read8(0xF8)<<8)|self._read8(0xF9))>>4
        raw_t = ((self._read8(0xFA)<<16)|(self._read8(0xFB)<<8)|self._read8(0xFC))>>4
        raw_h = (self._read8(0xFD) << 8)| self._read8(0xFE)
        return raw_t, raw_p, raw_h

    async def read_compensated_data(self):
        # noinspection PyBroadException
        try:
            raw_t, raw_p, raw_h = await self.read_raw_data()
        except:
            print(i2c_err_str.format(self.addr))
            return float('NaN'), float('NaN'), float('NaN')
        var1 = ((raw_t>>3)-(self._T1<<1))*(self._T2>>11)
        var2 = (raw_t >> 4)-self._T1
        var2 = var2*((raw_t>>4)-self._T1)
        var2 = ((var2>>12)*self._T3)>>14
        self._t_fine = var1+var2
        temp = (self._t_fine*5+128)>>8
        var1 = self._t_fine-128000
        var2 = var1*var1*self._P6
        var2 = var2+((var1*self._P5)<<17)
        var2 = var2+(self._P4<<35)
        var1 = (((var1*var1*self._P3)>>8)+
                ((var1*self._P2)<<12))
        var1 = (((1<<47)+var1)*self._P1)>>33
        if var1 == 0:
            pres = 0
        else:
            p = ((((1048576-raw_p)<<31)-var2)*3125)//var1
            var1 = (self._P9*(p>>13)*(p >> 13))>>25
            var2 = (self._P8*p)>>19
            pres = ((p+var1+var2)>>8)+(self._P7<<4)
        h = self._t_fine-76800
        h = (((((raw_h<<14)-(self._H4<<20)-
                (self._H5*h))+16384)
              >>15)*(((((((h*self._H6)>>10)*
                            (((h*self._H3)>>11)+32768))>>10)+
                          2097152)*self._H2+8192)>>14))
        h = h-(((((h>>15)*(h>>15))>>7)*self._H1)>>4)
        h = 0 if h < 0 else h
        h = 419430400 if h>419430400 else h
        humi = h>>12
        return temp, pres, humi

    async def values(self):
        temp, pres, humi = await self.read_compensated_data()
        return temp / 100, pres / 256, humi / 1024

    async def pressure_precision(self):
        p = (await self.read_compensated_data())[1]
        pi = float(p // 256)
        pd = (p % 256)/256
        return pi, pd

    async def altitude(self, pressure_sea_level=1013.25):
        pi, pd = await self.pressure_precision()
        return 44330*(1-((float(pi+pd)/100)/pressure_sea_level)**(1/5.255))
