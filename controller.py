import RPi.GPIO as GPIO
import time



class Controller():
	def __init__(self):
		GPIO.setmode(GPIO.BCM)
		self.__x_dir_pin = 22
		self.__x_step_pin =22
		self.__y_dir_pin = 22
		self.__y_step_pin = 22

		GPIO.setup(self.__x_dir_pin, GPIO.OUT)
		GPIO.setup(self.__x_step_pin, GPIO.OUT)
		GPIO.setup(self.__y_dir_pin, GPIO.OUT)
		GPIO.setup(self.__y_step_pin, GPIO.OUT)

	def move_stone(self, relative_x, relative_y):
		x_dir = GPIO.HIGH
		if relative_x < 0:
			x_dir = GPIO.LOW
			relative_x = - relative_x
		GPIO.output(self.__x_dir_pin, x_dir)

		y_dir = GPIO.HIGH
		if relative_y < 0:
			y_dir = GPIO.LOW
			relative_y = - relative_y
		GPIO.output(self.__y_dir_pin, y_dir)


		max_step = max(relative_x, relative_y)
		for i in range(0,max_step):
			if relative_x > 0:
				GPIO.output(self.__y_dir_pin, y_dir)
				relative_x -= 1

			if relative_y > 0:
				GPIO.output(self.y_step_pin, y_dir)
				relative_y -= 1

			time.sleep(0.2)
			GPIO.output(self.x_step_pin, y_dir)
			GPIO.output(self.y_step_pin, y_dir)
			time.sleep(0.2)
		

if __name__ == '__main__':

	testor = Controller()
	testor.move_stone(1,2)


# set the GPIO mode
# loop over the LEDs on the TrafficHat and light each one
# individually
# for i in (22, 23, 24):
# 	GPIO.setup(i, GPIO.OUT)
# 	GPIO.output(i, GPIO.HIGH)
# 	time.sleep(3.0)
# 	GPIO.output(i, GPIO.LOW)
# # perform a bit of cleanup
# GPIO.cleanup()
# print('end')
