import cv2

class NonVecRecorder():
	def __init__(self, env):
		self.env = env
		self.res = (1024, 1024)

	def init_video_writing(self, res=(1024,1024), fname=None):
		if fname == None:
			self.video = False
		else:
			self.video=True
			self.res = res
			#fourcc = cv2.VideoWriter_fourcc(*'XVID')
			fourcc = cv2.VideoWriter_fourcc(*'MP4V')
			self.out = cv2.VideoWriter(fname, fourcc, 5, self.res)

	def viz(self, render_img):
		if render_img == True:
			if not self.video:
				#self.env.render()
				print("not self.video") #not rendering since you can't render on a remote instance
			elif self.video:
				I = self.env.render(height=self.res[0], width=self.res[1], mode='rgb_array')
				#self.out.write(cv2.flip(I, 0))
				self.out.write(I)

	def save_video(self):
		self.out.release()

	def close(self):
		self.save_video() 