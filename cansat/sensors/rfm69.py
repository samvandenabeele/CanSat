from machine import SPI, Pin
from rfm69 import RFM69

class RFM69HCW:
	def __init__(self) -> None:
		self.spi = SPI(baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB, sck=4, mosi=7, miso=6)
		self.nss = Pin(5, Pin.OUT, value=True )
		self.rst = Pin(3, Pin.OUT, value=False )
		self.rfm = RFM69( spi=self.spi, nss=self.nss, reset=self.rst )
		self.rfm.frequency_mhz = 433.1
		self.rfm.encryption_key = ( b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08" )

	def transmit(self, data: str) -> bool:
		return self.rfm.send(bytes(f"{data}\r\n", "utf-8"))
		
	def dbm_to_mw(self, dbm: float) -> float:
		"""Convert dBm to mW."""
		return 10 ** (dbm / 10)

	def alive(self) -> bool:
		try:
			print('RFM version     :', self.rfm.version)
			print('Freq            :', self.rfm.frequency_mhz)
			print('Freq. deviation :', self.rfm.frequency_deviation, 'Hz')
			print('bitrate         :', self.rfm.bitrate, 'bits/sec')
			print('tx power        :', self.rfm.tx_power, 'dBm')
			print('tx power        :', self.dbm_to_mw(self.rfm.tx_power), 'mW')
			print('Temperature     :', self.rfm.temperature, 'Celsius')
			print('Sync on         :', 'yes' if self.rfm.sync_on else 'no')
			print('Sync size       :', self.rfm.sync_size)
			print('Sync Word Length:', self.rfm.sync_size + 1, "(Sync size+1)")
			print('Sync Word       :', self.rfm.sync_word)
			print('CRC on          :', self.rfm.crc_on)
			print('Preamble Length :', self.rfm.preamble_length)
			print('aes on          :', self.rfm.aes_on)
			print('Encryption Key  :', self.rfm.encryption_key)
			return True
		except:
			return False
