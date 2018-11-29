### EXAMPLE PYTHON MODULE
# Define some variables:
numberone = 1
ageofqueen = 78
# define some functions
def printhello():
    print('hello')
    
def timesfour(input):
    print( input * 4)
    
# define a class
class Piano:      
    def __init__(self, dogname, dob_d, dob_m, dob_y, dogSpeakText):
        self.name_of_dog = dogname
        self.date_of_birth = dob_d
        self.month_of_birth = dob_m
        self.year_of_birth = dob_y
        self.sound_it_make = dogSpeakText
    def printdetails(self):
        print(self.year_of_birth)