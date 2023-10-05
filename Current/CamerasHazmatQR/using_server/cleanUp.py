def removeSpecialCharacter(s):
    t = ""
    for i in s:
        if i >= 'A' and i <= 'Z' or i == " ":
            t += i
    return t