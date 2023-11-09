#!/usr/bin/env python
# -- coding: utf-8 --
from openai import OpenAI
import rospy
import speech_recognition as sr
import os
import time
from playsound import playsound
from gtts import gTTS
from konlpy.tag import Kkma
from std_msgs.msg import String
import re
import pygame
#rktkdghksrudtkdydgka /.venv
"""
간단 수정법

인식하고 싶은 물체를 tool_list에 적는다(한글)
그 물체에 대응되는 영어 단어를 같은 위치에 적는다
prepare_to_go_out list must be english
"""
system_message='너의 이름은 javas이고 친절한 개인 비서 로봇이야. 대답은 3줄 이하로 존대말로 해줘. 짧게 대답할 수 있는 말이면 1줄이나 2줄로 대답해도 괜찮아. 너는 로봇팔이 달려있는 자동차에 내장되어있어서 움직일수 있고 물건을 잡을 수 있어. 또 스피커도 내장되어 있어서 너가 하는 말은 스피커로 출력돼 그리고 너가 현재 위치한 곳은 원흥관 i-space이야. 추가로 매 답변 마지막에 답변하며 지을 적절한 표정을 (웃음),(놀람),(화남)중에 선택해서 출력해줘 그리고 출력된 감정은 이모티콘으로 바뀌어서 모니터 디스플레이를 통해 전달된다는걸 기억해'

tool_list = ["틀니", "틀니통" , "핸드폰", "물병","커피", "우산", "수건", "타월", "약", "약통",  "장바구니", "가방", "칫솔"]
en_tool_list = ["tooth","tooth","phone","bottle","bottle","umbrella","towel","towel","medicine","medicine","bag","bag","case"]
bring_list = ["갖","가져다주","갖다주","가지"]

open_list = ["열"]

prepare_to_go_out = ["hat","phone"]

kkma=Kkma()
file_name='sample.mp3'
client = OpenAI(api_key="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")
r = sr.Recognizer()
m = sr.Microphone()
turn_off_flag=False
recog_flag=True
rospy.init_node('listener')
ans_pub=rospy.Publisher("tool_list",String,queue_size=1)
ex_answer=''
is_1 = False
is_2 = False
is_3 = False

def find_text_between_parentheses(text):
    pattern = r'\((.*?)\)'  # 괄호 안의 문자열을 추출하는 정규 표현식
    matches = re.findall(pattern, text)
    return matches[0]

def extract_text_outside_parentheses(text):
    pattern = r'\([^)]*\)'  # 괄호 안의 내용을 추출하는 정규 표현식
    result = re.sub(pattern, '', text)
    return result

def gpt_ask(text):
    # try:
    # 대화 시작
    user_messages = [
        {"role": "system", "content": system_message},
        {"role": "user", "content": text}
    ]

    # OpenAI API 호출
    completion = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=user_messages
    )
    # completion = openai.ChatCompletion.create(
    #     model="gpt-3.5-turbo",
    #     messages=user_messages
    # )

    # API 응답에서 답변 텍스트 추출
    answer = completion.choices[0].message.content

    #답변에서 얼굴 추출
    face = find_text_between_parentheses(answer)
    if face == '놀람':
        face = '슬픔'
    #표정 제외한 답변
    result=extract_text_outside_parentheses(answer)
    answer=result.strip()
    return answer, face
    # except:
    #     return "죄송합니다. 다시 말씀해주세요","슬픔"

def callback(r, audio):
    global turn_off_flag, recog_flag, ex_answer
    answer=''
    text=''
    try:
        text = r.recognize_google(audio, language='ko')
        candidates = r.recognize_google(audio, language='ko', show_all=True)

        full_text = ''
        for text in candidates['alternative']:
            full_text = full_text + '. ' + text['transcript']
        text = text['transcript']
        print("[사용자] " + text)

        if '그만' in full_text:
            turn_off_flag=True
            face='슬픔'
            speaker("장치를 종료합니다",face)
        
        elif '굿모닝' in full_text or '좋은 아침' in full_text:
            ans_pub.publish("light_on")
            answer = '불을 킬게요'
            face = '웃음'
            speaker(answer,face)

        elif '나갈' in full_text:
            go_out_list = ' '.join(prepare_to_go_out)
            ans_pub.publish(go_out_list)
            answer = '네, 필요한 물건들을 가져올게요'
            face = '웃음'
            speaker(answer,face)         
            
        else:
            order_flag, spoken_tool=sentence_analysis(full_text)
            if order_flag is 0:
                answer=' '.join(spoken_tool)
                answer=answer+'를 가져올게요'
                en_tool_answer=[]
                for i in range(len(tool_list)):
                    if tool_list[i] in spoken_tool:
                        en_tool_answer.append(en_tool_list[i])

                words_str = ' '.join(en_tool_answer)
                ans_pub.publish(words_str)
                face='웃음'
                speaker(answer,face)

            elif order_flag is 1:
                answer='그 물건은 가져올 수 없어요'
                
                face='슬픔'
                speaker(answer,face)

            elif order_flag is 3:
                answer='문을 열어드릴게요'
                face='웃음'
                ans_pub.publish('door')
                speaker(answer,face)
            else:
                answer,face=gpt_ask(text)
                
                speaker(answer,face)

        recog_flag=True    
        ex_answer=answer
        print("[자바스] 듣고있어요")
    except sr.UnknownValueError:
        recog_flag=False
    except sr.RequestError as e:
        print(f"[자바스] 서버 연결에 실패하였습니다 : {e}")

def speaker(text,face):
    global is_1, is_2, is_3
    if face=='화남':
        is_3 = True
        is_2 = False
        is_1 = False
    elif face=='슬픔':
        is_3 = False
        is_2 = True
        is_1 = False
    else:
        is_3 = False
        is_2 = False
        is_1 = True       

    print("[자바스] ",text)
    tts_ko=gTTS(text=text,lang='ko')
    tts_ko.save(file_name)
    playsound(file_name)
    if os.path.exists(file_name):
        os.remove(file_name)


def sentence_analysis(sentence):
    '''문장에서 도구와 가져오라는 명령이 포함되면 Ture를 반환한다.'''
    tool_flag=False
    bring_flag=0
    open_flag = False
    pos_tags = kkma.pos(sentence)
    spoken_tool=[]
    #품사와 함께 반환
    vv_words = [word for word, pos in pos_tags if pos == 'VV']

    for tool in tool_list:
        if tool in sentence:
            spoken_tool.append(tool)
            tool_flag=True

    for word in vv_words:
        for bring in bring_list:
            if word == bring:
                bring_flag=True

    for word in vv_words:
        for open in open_list:
            if word == open:
                open_flag=True

    if open_flag is True:
        return 3, []

    if bring_flag is True and tool_flag is True:
        return 0, spoken_tool
    elif bring_flag is True and tool_flag is False:
        return 1, spoken_tool
    else:
        return 2, spoken_tool

pygame.init()

size = [800, 600]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("image_animation")
player_posX = 100
player_posY = 0



Count = 0
Count2 = 0

smile_face = [pygame.image.load("smile1.png"), pygame.image.load("smile2.png"), pygame.image.load("smile3.png")]
sup_face = [pygame.image.load("sup1.png"), pygame.image.load("sup2.png"), pygame.image.load("sup3.png")]
angry_face = [pygame.image.load("angry1.png"), pygame.image.load("angry2.png"), pygame.image.load("angry3.png")]

for i, image in enumerate(smile_face):
    smile_face[i] = pygame.transform.scale(image, (600, 600))
for i, image in enumerate(sup_face):
    sup_face[i] = pygame.transform.scale(image, (600, 600))
for i, image in enumerate(angry_face):
    angry_face[i] = pygame.transform.scale(image, (600, 600))

def playerMove():
    global Count, Count2, player_posX, player_posY

    screen.fill((0,0,0))

    if Count > 30:
        Count = 0
    
    if Count%10 == 9:
        Count2 +=1

    if Count2 > 3:
        Count2 = 0
    
    if Count2 == 0:
        walkCount = 0
    elif Count2 == 1:
        walkCount = 1
    elif Count2 == 2:
        walkCount = 2
    elif Count2 == 3:
        walkCount = 1

    if is_2 == True:
        if walkCount == 0:
            screen.blit(sup_face[walkCount], (player_posX+10, player_posY))
        else:
            screen.blit(sup_face[walkCount], (player_posX, player_posY))

    elif is_3 == True:
        if walkCount == 0:
            screen.blit(angry_face[walkCount], (player_posX+5, player_posY))
        else:
            screen.blit(angry_face[walkCount], (player_posX, player_posY))

    elif is_1 == True:
        if walkCount == 2:
            walkCount = 1
        if walkCount == 0:
            screen.blit(smile_face[walkCount], (player_posX+5, player_posY))
        else:
            screen.blit(smile_face[walkCount], (player_posX, player_posY-10))

    else:
        if walkCount == 2:
            walkCount = 1
        if walkCount == 0:
            screen.blit(smile_face[walkCount], (player_posX+5, player_posY))
        else:
            screen.blit(smile_face[walkCount], (player_posX, player_posY-10))
    
    Count+=1

clock = pygame.time.Clock()

flag = True

with m as source:
    r.adjust_for_ambient_noise(source)

stop_listening = r.listen_in_background(m, callback)

flag = 0
# print("[자바스] 인식을 시작합니다")
while turn_off_flag==False:
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_q:
                turn_off_flag = True
    playerMove()
    pygame.display.update()

    if flag == 0:
        print("[자바스] 인식을 시작합니다")
        flag = 1
pygame.quit()

        
        

