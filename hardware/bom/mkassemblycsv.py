

import BOM

optionset = set(['DNI'])

## 'NOASSEMBLY' to include parts needed by not put on by assembly house

bom = BOM.BOM("../radioberry-preamp.xml",optionset=optionset)


bom.CSVAssemblyPrint(['Mouser','Digi-Key'])


