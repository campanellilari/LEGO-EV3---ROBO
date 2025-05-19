#!/usr/bin/env pybricks-micropython

#Importando pacotes necessários para a programação dos 
#componentes do kit lego Ev3 Mindstormes Education
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch

#Declaração da Classe LegoTech
class LegoTech():

    #Constantes declaradas para uso em 
    #qualquer instância da classe no
    #programa
    GYRO_CALIBRATION_LOOP = 200
    GYRO_OFFSET_FACTOR = 0.001
    TARGET_LOOP_PERIOD = 10
    OBSTACLE_DISTANCE = 250

    #Inicialização de atributos da classe
    #que serão utilizadas no programa
    def __init__(self):
        self._ev3 = EV3Brick()
        self._left_motor = Motor(Port.D)
        self._right_motor = Motor(Port.A)
        self._gyro_sensor = GyroSensor(Port.S2)
        self._ultra_sensor = UltrasonicSensor(Port.S4)
        self._fall_timer = StopWatch()
        self._single_loop_timer = StopWatch()
        self._control_loop_timer = StopWatch()
        self._average_control_loop_period = 0
        ####################################
        self._motor_position_sum = 0
        self._control_loop_count = 0
        self._motor_position_change = [0, 0]
        self._robot_body_angle = -0.25
        self._wheel_angle = 0
        self._drive_speed = 0
        self._steering = 0
        self._gyro_offset = 0
        
    #Declaração do Método para reiniciar
    #os valores de alguns dos atributos
    def reset_atributes(self):
        self._left_motor.reset_angle(0) 
        self._right_motor.reset_angle(0)
        self._fall_timer.reset()
        
        self._motor_position_sum = 0
        self._control_loop_count = 0
        self._motor_position_change = [0, 0]
        self._robot_body_angle = -0.25
        self._wheel_angle = 0
        self._drive_speed = 0
        self._steering = 0        
        
    #Declaração de Método para calibrar
    #o sensor giroscópio tirando um valor
    #médio
    def calibrate_gyro(self):
        while True:
            gyro_min_rate = 440
            gyro_max_rate = -440
            gyro_sum = 0
            for _ in range(self.GYRO_CALIBRATION_LOOP):
                gyro_value = self._gyro_sensor.speed()
                gyro_sum += gyro_value
                gyro_min_rate = min(gyro_min_rate, gyro_value)
                gyro_max_rate = max(gyro_max_rate, gyro_value)
                wait(5)
            if (gyro_max_rate - gyro_min_rate) < 2:
                break
        self._gyro_offset = gyro_sum / self.GYRO_CALIBRATION_LOOP

    #Declaração do método para exibir diferentes
    #faces no display do lego brick
    def display(self, face):
        self._ev3.screen.load_image(face)

    #Declaração do método para tocar sons
    #no lego brick
    def speak(self, sound):
        self._ev3.speaker.play_file(sound)
        
    #Declaração de método para detectar
    #objetos e desviar utilizando 
    #o sensor ultrassônico
    def _ultra_detection(self):
        if self._ultra_sensor.distance() < self.OBSTACLE_DISTANCE:
            self._drive_speed = -100
            self._steering = 0
        else:
            self._drive_speed = 0
            self._steering = 0

    #Declaração do método para atualizar o tempo
    #corrido e controle do loop principal
    def adjust_loop_timer(self):
        self._single_loop_timer.reset()
        
        if (self._control_loop_count == 0):
            self._average_control_loop_period = self.TARGET_LOOP_PERIOD / 1000
            self._control_loop_timer.reset()
        else:
            self._average_control_loop_period = (
                self._control_loop_timer.time() / 
                1000 / 
                self._control_loop_count
            )
        self._control_loop_count += 1

    #Declaração do método para calcular
    #a taxa de variação e angulação
    #do giroscópio
    def _get_robot_body_rate(self):
        gyro_value = self._gyro_sensor.speed()
        self._gyro_offset *= (1 - self.GYRO_OFFSET_FACTOR)
        self._gyro_offset += self.GYRO_OFFSET_FACTOR * gyro_value
        robot_body_rate = gyro_value - self._gyro_offset
        self._robot_body_angle += robot_body_rate * self._average_control_loop_period
        return robot_body_rate

    #Declaração do método para calcular
    #a taxa de variação e angulação
    #dos motores 
    def _get_wheel_rate(self):
        left_motor_angle = self._left_motor.angle()
        right_motor_angle = self._right_motor.angle()
        previous_motor_sum = self._motor_position_sum
        self._motor_position_sum = left_motor_angle + right_motor_angle
        change = self._motor_position_sum - previous_motor_sum
        self._motor_position_change.insert(0, change)
        del self._motor_position_change[-1]
        self._wheel_angle += change - self._drive_speed * self._average_control_loop_period
        wheel_rate = (
                sum(self._motor_position_change) / 
                len(self._motor_position_change) /
                self._average_control_loop_period
        )
        return wheel_rate
        
    #Declaração do método para calcular
    #o output enviado para os motores
    def _get_output(self):
        robot_body_rate = self._get_robot_body_rate()
        wheel_rate = self._get_wheel_rate()
        
        output_power = (
            (-0.01 * self._drive_speed) + (
                1.2 * robot_body_rate +
                20 * self._robot_body_angle +
                0.1 * wheel_rate +
                0.15 * self._wheel_angle
            )
        )
        output = max(min(output_power, 100), -100)
        return output
        
    #Declaração do método responsável
    #por detectar se o robô caiu
    def _has_fallen(self, output):
        if abs(output) < 100:
            self._fall_timer.reset()
            return False
        elif self._fall_timer.time() > 1000:
            return True
        
        return False        

    #Declaração do método que enviará os
    #valores do output para os motores
    def try_to_balance(self):
        output = self._get_output()
        
        self._left_motor.dc(output - 0.1 * self._steering)
        self._right_motor.dc(output + 0.1 * self._steering)

        if(self._has_fallen(output)):
            return True
        
        self._ultra_detection()
        return False

    #Declaração do método de espera
    #do programa
    def get_loop_timing(self):
        return self.TARGET_LOOP_PERIOD - self._single_loop_timer.time()

    #Declaração do método responsável por
    #reiniciar o robo loop principal
    def restart_robot(self):
        self._left_motor.stop()
        self._right_motor.stop()
        self.display(ImageFile.DIZZY)
        self.speak(SoundFile.SPEED_DOWN)
        wait(3000)
            
#Função principal do projeto, na qual
#irá instânciar um objeto da classe
#LegoTech e irá rodar o programa
def main():
    legotech = LegoTech()

    while True:
        
        legotech.display(ImageFile.SLEEPING)
        
        legotech.reset_atributes()
        legotech.calibrate_gyro()

        legotech.speak(SoundFile.SPEED_UP)
        legotech.display(ImageFile.ANGRY)
        
        legotech.speak(SoundFile.SPEED_IDLE)

        while True:
            legotech.adjust_loop_timer()
            
            if(legotech.try_to_balance()):
                break
                
            wait(legotech.get_loop_timing())

        legotech.restart_robot()

if __name__ == "__main__":
    main()