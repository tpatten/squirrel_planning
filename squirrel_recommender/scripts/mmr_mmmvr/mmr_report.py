######################
## Version 0.1 #######
######################

## #############################################################
def mmr_report(ctitle,xresulttra,xresulttes,xpr):

  print('***** '+ctitle+' *****')
  print('Mean and std of the accuracy on train + error')
  print(xresulttra)
  print('Mean and std of the accuracy on test + error')
  print(xresulttes)
  print('Precision, recall, f1')
  print(xpr)



