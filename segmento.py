

class Objetivo:
    def __init__(self):
        self.pInicio = (0, 0)
        self.pFin = (0, 0)
        self.type = 1 # 1 segmento, 2 triángulo
        self.pMedio = (0, 0)

    def setInicio(self, inicio):
        self.pInicio = inicio
    
    def setFin(self, fin):
        self.pFin = fin
    
    def setMedio(self, medio):
        self.pMedio = medio
        self.type = 2

    # Devuelve 1 si se trata de un segmento y 2 si es un triángulo
    def getType(self):
        return self.type

    # Obtiene el punto de inicio del segmento
    def getInicio(self):
        return self.pInicio
    
    # Obtiene el punto final del segmento
    def getFin(self):
        return self.pFin
    
    # Obtiene el punto medio que forma el triángulo
    def getMedio(self):
        return self.pMedio
