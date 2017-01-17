#!/usr/bin/env python

import rospy

from communication_node.msg import comm_msg


def menu():
    print("Menu:")
    print("1 - menu")
    print("2 - plataforma")
    print("3 - teste radio (pvariant test)")
    print("4 - radio config")
    print("5 - verificar se existe mensagem")
    print("6 - mandar mensagem")
    print("7 - ler mensagem")
    print("8 - mandar velocidades")
    print("9 - repete a ultima velocidade")
    print("0 - para o robo")
    try:
        op = input('Selecione a opcao ')
        return op
    except EOFError:
        exit(1)

def getmotorVel(motor):
    try:
        motorvel = input("Velocidade Motor {}: ".format(motor))
    except EOFError:
        print("")
        exit(1)
    else:
        return motorvel

def publisher():
    pub = rospy.Publisher('radio_topic', comm_msg, queue_size=10)
    rospy.init_node('radio_topic', anonymous=True)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            msg = comm_msg()
            msg.menu = menu()
            if msg.menu == 8:
                msg.MotorA = getmotorVel("A")
                msg.MotorB = getmotorVel("B")
            if msg.menu == 9:
                pass
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        exit(1)


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
