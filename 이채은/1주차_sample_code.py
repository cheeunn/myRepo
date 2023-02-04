import random

class Dicegame:
    def __init__(self):
        self.N = int(input())

        while (self.N <= 10 or self.N >= 50):
            print("조건에 맞지 않습니다. 다시 입력하세요.")
            self.N = int(input())
        
        self.dice_num = [0, 0, 0, 0, 0, 0]



    # random 라이브러리를 활용해서 주사위를 N번 굴려 결과값(각각의 수가 몇번 나왔는지)을 저장하는 함수
    def roll_dice(self):
        """
        주사위를 N번 굴려 결과값들을 저장하는 코드를 작성해주세요. 
        """
        if cnt <= self.N
            rannom = random.random(1,7) 
            self.dice_num[rannom] += 1



    # 1. 저장된 결과값들 2. 두번째로 많이 나온 주사위 수 를 print하는 함수
    def print_result(self):
        """
        1. 결과값(각각의 수가 몇번 나왔는지)을 모두 print하는 코드
        2. 두번째로 많이 나온 주사위 수를 print하는 코드

        위의 1, 2번을 print하는 코드를 작성해주세요.
        """
        for i in 6 :
            print('Dice number ', i+1,' : ', self.dice_num[i], end = '\n')

        sel.dice_num.sort(reverse=True)




if __name__ == "__main__":
    dice_game = Dicegame()