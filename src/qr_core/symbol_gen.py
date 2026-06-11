class SymbolGenerator:
    """
    Generate new symbols guaranteed to be different from one another
    Optionally, supply a prefix for mnemonic purposes
    Call gensym("foo") to get a symbol like 'foo37'
    """
    def __init__(self):
        self.count = 0

    def gensym(self, prefix = 'i', zero_padded = False):
        self.count += 1
        if zero_padded:
            return (prefix + '_%08i') % self.count
        else:
            return prefix + '_' + str(self.count)

gensym = SymbolGenerator().gensym
"""Call this function to get a new symbol"""
