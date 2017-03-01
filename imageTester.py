#!/usr/bin/env python
import os, glob, shutil, subprocess

''' 
*** Benötigte Ordner werden vom Skript erstellt und der results bzw. diffs Ordner werden vor der Ausführung geleert
*** 
*** Zu testende Bilder müssen im Ordner tests liegen und den Namen <referenzBild>_XXX.jpg besitzen
*** Die Differenzbilder werden in Ordner diffs gespeichert und haben den gleichen Namen wie das getestete Bild
*** Die erzeugten QR-COdes werden im Ordner results gespeichert und haben den gleichen Namen wie das getestete Bild
*** Die Referenzbilder müssen im Ordner references liegen
'''

executable = 'qr_detection'
baseDir = 'tests'

refDir = 'references'
resDir = 'results'
imgDir = 'images'
difDir = 'diffs'

def testImages():
    testImageRegex = os.path.join(baseDir, imgDir, '*.jpg')
    testImages = glob.glob(testImageRegex)

    correct = 0

    for img in testImages:
        path, name = os.path.split(img)
        
        res = os.path.join(baseDir, resDir, name)
        
        refName = name[:name.rfind('_')] + '.png'
        ref = os.path.join(baseDir, refDir, refName)
        
        resCode = subprocess.call(['./' + executable, img, res, ref])
        
        if resCode == 0:
            correct += 1
            
            diffPath = os.path.join(baseDir, difDir, name.replace('jpg', 'png'))
            shutil.move("diff.png", diffPath)
    
    return len(testImages), correct


if __name__ == '__main__':
    
    path = os.path.join(baseDir, difDir)
    if os.path.exists(path): shutil.rmtree(path)
    
    path = os.path.join(baseDir, resDir)
    if os.path.exists(path): shutil.rmtree(path)
    
    path = os.path.join(baseDir, refDir)
    if not os.path.exists(path): os.makedirs(path)
    
    path = os.path.join(baseDir, resDir)
    if not os.path.exists(path): os.makedirs(path)
    
    path = os.path.join(baseDir, imgDir)
    if not os.path.exists(path): os.makedirs(path)
    
    path = os.path.join(baseDir, difDir)
    if not os.path.exists(path): os.makedirs(path)
    
    anz, correct = testImages()
    
    print('\nEs wurden %d Bilder getestet' %(anz))
    if anz > 0: print('Davon wurden %d QR-Codes richtig erkannt (%.2f%%)' %(correct, correct/anz*100))
