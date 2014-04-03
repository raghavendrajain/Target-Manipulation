import csv
# import os
# os.remove("output.csv")


f = open('example.csv', 'rb')
reader=csv.reader(f)
headers=reader.next()

resultFile = open("output.csv",'w+')
resultFile.truncate()
wr = csv.writer(resultFile)
# wr.writerow(headers)


with open('example.csv', 'rb') as csvfile:
    for line in csv.reader(csvfile, skipinitialspace=True):
        # print line
        # print len(line)
        if (len(line)!=16):
        	del line
        else:
        	# resultFile = open("output.csv",'a')
        	wr = csv.writer(resultFile)
        	wr.writerow(line)






