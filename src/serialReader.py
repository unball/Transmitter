import serial

class SerialReader:
    def __init__(self, src, baudrate):
        self.serial = serial.Serial(src, baudrate)
        self.serial.timeout = 0.100
    
    def read(self):
        try:
            # Tenta ler uma linha
            line = self.serial.readline().decode()

            # Separa os dados
            splitted = line.replace('\n','').split(" ")

            # Converte os dados para float
            values = [float(x) for x in splitted]

            # Retorna
            return values
        
        except:
            # Não recebeu dado
            return None