#!/usr/bin/env python3
import os, glob, shutil, subprocess

''' 
*** Benoetigte Ordner werden vom Skript erstellt und der results/diffs/errors Ordner werden vor der Ausfuehrung geleert
*** 
*** Zu testende Bilder muessen im Ordner tests liegen und den Namen <referenzBild>_XXX.jpg besitzen
*** Die Differenzbilder werden in Ordner diffs gespeichert und haben den gleichen Namen wie das getestete Bild
*** Die erzeugten QR-Codes werden im Ordner results gespeichert und haben den gleichen Namen wie das getestete Bild
*** Bilder, auf welchen kein QR-Code erkannt wurden werden im Ordner errors abgelegt
*** Die Referenzbilder muessen im Ordner references liegen
'''

executable = 'qr_detector'
diffName = 'diff.png'
baseDir = 'tests'

dirs = {'ref':'references', 'res':'results', 'img':'images', 'err':'errors', 'dif':'diffs'}

def testImages():
    testImageRegex = os.path.join(baseDir, dirs['img'], '*.jpg')
    testImages = glob.glob(testImageRegex)

    stats = { 'all' : len(testImages) }
    stats['notFound'] = 0
    stats['correct'] = 0
    stats['failed'] = 0

    for img in testImages:
        path, name = os.path.split(img)
        
        res = os.path.join(baseDir, dirs['res'], name)
        
        refName = name[:name.rfind('_')] + '.png'
        ref = os.path.join(baseDir, dirs['ref'], refName)
        
        print("------------------- " + img + " -------------------")
        resCode = subprocess.call(['./' + executable, img, res, ref])

        if resCode == 0 and not os.path.isfile(diffName):
            stats['correct'] += 1
        
        elif resCode == 0: # Diff file exists
            stats['failed'] += 1
            diffPath = os.path.join(baseDir, dirs['dif'], name.replace('jpg', 'png'))
            shutil.move("diff.png", diffPath)
        
        else: # QR-Code was not found
            stats['notFound'] += 1
            errPath = os.path.join(baseDir, dirs['err'], name)
            shutil.copyfile(img, errPath)
            
        print()
        
        if os.path.isfile(diffName):
            os.remove(diffName)
            
    return stats


if __name__ == '__main__':
    
    path = os.path.join(baseDir, dirs['dif'])
    if os.path.exists(path): shutil.rmtree(path)
    
    path = os.path.join(baseDir, dirs['res'])
    if os.path.exists(path): shutil.rmtree(path)
    
    path = os.path.join(baseDir, dirs['err'])
    if os.path.exists(path): shutil.rmtree(path)
    
    for k,v in dirs.items():
        path = os.path.join(baseDir, v)
        if not os.path.exists(path): os.makedirs(path)
    
    stats = testImages()
    
    print('\nEs wurden %d Bilder getestet' %(stats['all']))
    if stats['all'] > 0:
        print('%d QR-Codes komplett richtig erkannt (%.2f%%)' %(stats['correct'], stats['correct'] / stats['all'] * 100))
        print('%d QR-Codes teilweise richtig erkannt (%.2f%%)' %(stats['failed'], stats['failed'] / stats['all'] * 100))
        print('In %d Bildern wurde kein QR-Codes gefunden (%.2f%%)' %(stats['notFound'], stats['notFound'] / stats['all'] * 100))
