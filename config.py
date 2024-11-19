class Config:
    def __init__(self, cfgFile):
        self._config = dict()
        with open(cfgFile) as f:
            for line in f.readlines():
                raw = line.strip()
                cmtIdx = raw.find('#')
                if cmtIdx != -1: raw = raw[0:cmtIdx].strip()
                if raw == '': continue
                eqIdx = line.find('=')
                if eqIdx == -1: raise Exception("Error on line : Not a key-value pair.")
                key = raw[0:eqIdx].strip()
                value = raw[eqIdx + 1:].strip()
                if self._config.get(key) != None: raise Exception("Error on line : Duplicate key '{}'.".format(key))
                else:
                    self._config[key] = value
    def Get(self, key):
        return self._config.get(key)
    def GetAs(self, key, type):
        if type is bool:
            return self._config.get(key) == 'True'
        return type(self._config.get(key))

