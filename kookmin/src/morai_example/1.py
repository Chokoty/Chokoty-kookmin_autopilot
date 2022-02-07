import sys
input = sys.stdin.readline

f1 = open("path/path_raw.txt", 'r')
f2 = open("path/path_edit.txt", 'w')

while True:
    line = f1.readline().split()
    if not line: break

    for i in range(3):
      if line[i][0]=='-' or line[i][2]=='0':
        line[i]=line[i][:7]
      else:
        line[i]=line[i][:6]

    # line[0] = '4.5'
    line[1] = '-5'
    # line[2] = '4.5'
    
    data = " ".join(line) 
    data = data +"\n"
    print(data)
    f2.write(data)

f1.close()
f2.close()
