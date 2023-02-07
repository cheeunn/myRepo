import random

class Dicegame:
    def __init__(self):
        self.N = int(input())

        while (self.N <= 10 or self.N >= 50):
            print("조건에 맞지 않습니다. 다시 입력하세요.")
            self.N = int(input())
        
        self.dice_num = [0, 0, 0, 0, 0, 0]


    # 1) Throw dice as many times as you have entered
    # 2) Save the number of times the dice's eyes appear in the list
    def roll_dice(self):
        for cnt in range(self.N):
            rannom = random.randrange(1,7) 
            self.dice_num[rannom-1] += 1
            cnt += 1


    def print_result(self):
        # 1) print the number of times of the eyes of the dice have come out
        last_index = len(self.dice_num) - 1
        index = 0

        for index in range(last_index+1):
            print('Dice number {} : {}'.format(index+1, self.dice_num[index]) )

        # find the second largest 
        sorted_list = sorted(self.dice_num,reverse=True)
        second_largest = list(set(sorted_list))[1]

        # 2) print the number of eyes of dice which came out 2nd mostly
        for index in range(last_index+1):
            if self.dice_num[index] == second_largest:
                print('Second Largest Number : {}'.format(index+1))


if __name__ == "__main__":
    dice_game = Dicegame()
    dice_game.roll_dice()
    dice_game.print_result()
