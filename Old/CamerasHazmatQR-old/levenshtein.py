def findDistance(word1, word2):
    word1=word1.lower()
    distances = []
    for x in range(len(word1)+1):
        distances.append([])
        distances[x].append(x)
    for i in range(len(word2)+1):
        if i != 0:
            distances[0].append(i)
    for w1 in range(1, len(word1)+1):
        for w2 in range(1, len(word2)+1):
            if word1[w1-1] == word2[w2-1]:
                value = distances[w1-1][w2-1]
                distances[w1].append(value)
            else:
                a = distances[w1][w2-1]
                b = distances[w1-1][w2]
                c = distances[w1-1][w2-1]
                value = (min(a, b, c)) + 1
                distances[w1].append(value)
    return distances[w1][w2]


def checkList(myWord,list):
    distances = {}
    for word in list:
        distance=0
        distance = findDistance(myWord, word)
        distances.update({word: distance})
    closest, distance = min(distances.items(), key=lambda x: x[1]) 
    return closest, distance