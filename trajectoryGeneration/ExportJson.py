import pickle
import json
import os


def WriteData(obj, path, name):
    with open(path + name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)


def ReadData(path, name):
    with open(path + name + '.pkl', 'rb') as f:
        return pickle.load(f)
    return object

def WriteJson(path,name,obj):
    with open(path + name + '.json','w') as fp:
        json.dump(obj,fp)

def ReadJson(path,name):
    obj = None
    with open(path + name + '.json','r') as fp:
        obj = json.load(fp)

    return obj

def StringifyKeysOfDictionary(inputDict):
    outputDict=dict()
    for key in inputDict:
        keyString = repr(key)
        keyString = keyString[1:]
        keyString = keyString[:-1]
        outputDict[keyString]=inputDict[key]

    return outputDict

# Convert a dictionary to a list
def ListifyDictionary(inputDict):
    outputList = []
    for key in inputDict:
        outputList.append(inputDict[key])

    return outputList


# JSON save:
#
# import json
#
# with open('data.json', 'w') as fp:
#     json.dump(data, fp)
#
# Supply extra arguments like sort_keys or indent to get a pretty result. The argument sort_keys will sort the keys alphabetically and indent will indent your data structure with indent=N spaces.
#
# json.dump(data, fp, sort_keys=True, indent=4)
#
# JSON load:
#
# with open('data.json', 'r') as fp:
#     data = json.load(fp)


if __name__ == "__main__":

    path = os.getcwd()
    
    # Currently
    # For each speed, multiple trajectories.
    # Each trajectory is a list of tuples
    actionPath = path + '/Actions/'
    
    completeActionsList = []
    for headingDict in completeActions:
        completeActionsList.append(ListifyDictionary(headingDict))
    
    if completeActionsList:
        WriteJson(actionPath,actionName,completeActionsList)
    
    
    actionsTemp = ReadJson(actionPath,actionName)
    actionsBack = []
    for actionHeadingList in actionsTemp:
        tempDict = dict()
        i=0
        for path in actionHeadingList:
            tempDict[i]=path
            i=i+1
    
        actionsBack.append(tempDict)
