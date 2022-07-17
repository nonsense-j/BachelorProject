# -*- coding: utf-8 -*-
import glob
import shutil

def main():
    txt = glob.glob('*.aut')
    #print txt
    changeList = []
    for a in txt:
        Name ,AUT  = a.split(".")
        NameList = Name.split("_")
        if NameList[-1] != 'bak':
            if Name+'_bak.aut' not in txt:
                shutil.copyfile(a, Name+"_bak.aut")
                changeList.append(a)
                aut = open(Name+ '_bak.aut')
                newAut = open(a,'w')
                line = aut.readline()
                while line:
                    if line.startswith('State'):
                        lineList = line.split(' ')
                        lineList[4] = lineList[1]
                        newLine = ' '.join(lineList)
                        #print newLine
                        newAut.write(newLine)
                    else:
                        newAut.write(line)
                    line = aut.readline()
                aut.close()
                newAut.close()
            else:
                print Name+'_bak.aut '+"already exists, "+a+" doesn't change, please check!"
    for c in changeList:
        print c +" modified successfully"

if __name__ == "__main__":
    main()