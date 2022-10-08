import csv

# 
# ROBOFLOW export to Tensorflow csv isn't compatible with Google Gloud's AutoML ui import
# e.g.
# Thessaloniki.jpg,640,360,car,302,166,400,242
#  should result in
# 302/640 = 0.471875
# 166/360 = 0.461111
# 400/640 = 0.625
# 242/360 = 0.672222 
#
# TEST,gs://my_ai_datasets/dashcam_objectdetection_v2_csv/Thessaloniki.jpg,car,0.471875,0.461111,,,0.625,0.672222,,
#


gsbacketpath = "gs://my_ai_datasets/dashcam_objectdetection_v2_csv/"

with open('dashcam_objectdetection_v2_csv_combined_annotations.csv', newline='') as csvfilein:
    spamreader = csv.reader(csvfilein, delimiter=',')
    with open('output.csv', 'w', newline='') as csvfileout:
        spamwriter = csv.writer(csvfileout, delimiter=',')
        rownum = 0
        problemmaticrows = []
        for row in spamreader:
            rownum = rownum + 1
            if len(row) == 0:
                continue            
            xSize = float(row[1])
            ySize = float(row[2])
            x1 = float(row[4]) / xSize
            y1 = float(row[5]) / ySize
            x2 = -1.0
            y2 = -1.0
            try:                
                x2 = float(row[6]) / xSize
                y2 = float(row[7]) / ySize
            except:
                # there might not be a 7,8 position 
                # it means the x2,y2 is the bottom right position of the image
                # thus:
                x2 = 1.0
                y2 = 1.0
            #x1str = '{:.2f}'.format(x1)
            try:
                #print(', '.join(row))
                gspath = gsbacketpath + row[0]
                spamwriter.writerow(['UNASSIGNED',gspath,row[3],'{:.2f}'.format(x1),'{:.2f}'.format(y1),'','','{:.2f}'.format(x2),'{:.2f}'.format(y2),'',''])
            except:
                print(rownum)
                print(', '.join(row))
                problemmaticrows.append(str(rownum))
            
    with open('outputproblem.csv', 'w', newline='') as csvfileproblemout:
        spamwriter = csv.writer(csvfileproblemout, delimiter=',')   
        for problemrow in problemmaticrows:
            spamwriter.writerow(problemrow[0])
