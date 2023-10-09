class Toggler:
    def __init__(self, start_state=False):
        self.state = start_state

    def toggle(self):
        self.state = not self.state

    def get(self):
        return self.state
    
    def __bool__(self):
        return self.state
    
    def __str__(self):
        return str(self.state)


def remove_dups(list, comp):
    new_list = []
    for item in list:
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list

def removeSpecialCharacter(s):
    t = ""
    for i in s:
        if i >= 'A' and i <= 'Z' or i == " ":
            t += i
    return t