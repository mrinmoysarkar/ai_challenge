



# delta = 1
# delxp = 0
# delxn = 0
# for i in range(4):
    # if i%2==0:
        # delxp += delta
        # x = delxp
    # else:
        # delxn -= delta
        # x = delxn
    # delyp = 0
    # delyn = 0
    # for j in range(4):
        # if j%2==0:
            # delyp += delta
            # y = delyp
        # else:
            # delyn -= delta
            # y = delyn
        # print(x,y)
        
minidel = 1
maxdel = 2
x = 0
y = 0
for i in range(6):
    if i%2 == 0:
        if y == 0:
            option = 1
        else:
            option = 2
        for j in range(6):
            #if j != 5:
            if option == 1:
                y += minidel
            else:
                y -= minidel
            print(x,y)
    else:
        for j in range(2):
            x += minidel
            print(x,y)
        