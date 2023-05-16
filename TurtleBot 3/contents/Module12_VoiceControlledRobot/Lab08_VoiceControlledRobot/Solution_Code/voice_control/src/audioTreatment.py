#!/usr/bin/env python

"""
	Program to :
		- capture a continuous stream from a standard microphone plugged in the computer using jack / usb / etc...
		- find words and phrases in this audio stream, then publish them in order to control the robot
		- can use either google's speech recognition API (online) or Firefox/Beidu Deepspeech API (offline)

	arguments :
		- samplerate : int : number of samples captured in one unit of time (one second)
		- duration : int : duration of each audio sample that will be processed by speech recognizer
		- mode : specifies speech recognition method. allowed modes :
			- "google" : use google's speech recognition API : sound is processed online so you need a permanent internet connection
			- "deepspeech" : use mozilla/baidu 's speech recognition API : works offline but you need to download or create a model to use it. base on recurrent neural networks
			- "CMUSphinx" : free and open source,  use "old" methods as gaussian mixture model, language model to process sound
"""

# System imports
import os, sys, time

# ROS import
import rospy
from std_msgs.msg import String

# audio import
import pyaudio, soundfile, sounddevice, speech_recognition, deepspeech

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

from deepspeech import Model

import os

print os.environ


"""
	MAIN CLASS

	this class allows to:
		- define the audio input parameters (samplerate, duration) using arguments
		- define the spech recognition API that will be used (google, pocketsphinx, deepspeech) using arguments
		- create a node to work in the ROS environment
			- define a publisher on the topic "speechRecognizer/outputString" to send words/sentences recognized as output
		- capture audio from a recognized microphone (USB / jack) and process speech recognition with the chosen API
		
	input : Microphone plugged into the computer (in a valid audio input port)
	output : words/sentences as a String, published on the topic "speechRecognizer/outputString"

"""
class audioTreatment(object):

	def __init__(self, audio_samplerate = 16000, audio_duration = 2):

		# algrithms used : you can add some if you want (but need to implement the code too)
		self.modeslist = ["deepspeech", "google", "sphinx"]

		# update audio parameters depending on the input
		self.updateAudioParams(audio_samplerate, audio_duration)

		# init ros node and parameters
		rospy.loginfo("Initializing ROS node 'audioTreatment'")
		rospy.init_node("audioTreatment", anonymous = True)
		rospy.loginfo("Successfully created 'audioTreatment' node")
		rospy.on_shutdown(self.shutdown)

		# publisher initialisation
		rospy.loginfo("Initializing String recognized publisher")
		self.stringPublisher = rospy.Publisher('speechRecognizer/outputString', String, queue_size=1)
		rospy.loginfo("Successfully initialized publisher 'speechRecognizer/outputString'")

		# check if arguments are specified
		_mode_param = "~mode"
		_samplerate_param = "~samplerate"
		_duration_param = "~duration"

		if rospy.has_param(_mode_param):
			self.mode = rospy.get_param(_mode_param)
			if (self.mode not in self.modeslist):
				rospy.logerr("Argument {} does not exist !\nModes available : {}".format(_mode_param, *self.modeslist))
				sys.exit()
		else:
			self.mode = "deepspeech"

		if rospy.has_param(_samplerate_param):
			self.audio_samplerate = rospy.get_param(_samplerate_param)
		if not (isinstance(self.audio_samplerate, int)):
			rospy.logerr("samplerate argument is not valid, it must be of type INT")
			sys.exit()

		if rospy.has_param(_duration_param):
			self.audio_duration = rospy.get_param(_duration_param)
		if not (isinstance(self.audio_duration, int)):
			rospy.logerr("audio duration argument is not valid, it must be of type INT")
			sys.exit()

		self.updateAudioParams(self.audio_samplerate, self.audio_duration)

		# init mode parater depending on the mode chosen
		self.initModes(self.mode)
		# run audioTreatment. Program will be stuck in this function and run in a loop
		self.runAudioTreatment(self.mode)
	pass

	#---------------------------------------------------------------------------------------------------------------------------------------
	# function to update audio characteristics : frequency, duration, number of samples ...	
	#---------------------------------------------------------------------------------------------------------------------------------------
	def updateAudioParams(self, audio_samplerate, audio_duration):
		self.audio_samplerate = audio_samplerate
		self.audio_sampleperiod = 1./self.audio_samplerate
		self.audio_nyquistrate = self.audio_samplerate / 2
		self.audio_nyquistperiod = 1./self.audio_nyquistrate
		self.audio_duration = audio_duration
		self.audio_numsamples = self.audio_duration * self.audio_samplerate
	pass

	#---------------------------------------------------------------------------------------------------------------------------------------
	# function to define speech recognition API 's parameters depending on the mode chosen	
	#---------------------------------------------------------------------------------------------------------------------------------------
	def initModes(self, mode):
		# using deepspeech speech recognition API
		if (mode == "deepspeech"):
			rospy.logwarn("Deepspeech (mozilla/beidu) API chosen\nInitializing API parameters")

			# define global constants : if you use the standard mozilla trained model (en-us), keep the next values (default)
			self.deepspeech_beam_width = 500
			self.deepspeech_lm_alpha = 0.75
			self.deepspeech_lm_beta = 1.85
			self.deepspeech_n_features = 26
			self.deepspeech_n_context = 9

			# alphabet model file path : please change it according to your directory location
			self.deepspeech_alphabet = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/deepspeech/deepspeech-0.5.1-models/alphabet.txt"
			if not (os.path.exists(self.deepspeech_alphabet)):
				rospy.logerr("Alphabet file does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()
			rospy.loginfo("Deepspeech alphabet successfully set")

			# Language model file path : please change it according to your file location
			self.deepspeech_languageModel = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/deepspeech/deepspeech-0.5.1-models/output_graph.pbmm"
			if not (os.path.isfile(self.deepspeech_languageModel)):
				rospy.logerr("Langage model diretory does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()
			rospy.loginfo("Deepspeech output graph successfully set")

			# trie file path : please change it according to your file location
			self.deepspeech_trie = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/deepspeech/deepspeech-0.5.1-models/trie"
			if not (os.path.isfile(self.deepspeech_trie)):
				rospy.logerr("trie file does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()
			rospy.loginfo("Deepspeech trie successfully set")

			# initialize DeepSpeech recognizer
			rospy.loginfo("Initializing recognizer")
			rospy.loginfo(self.deepspeech_languageModel)
			rospy.loginfo(self.deepspeech_n_features)
			rospy.loginfo(self.deepspeech_n_context)
			rospy.loginfo(self.deepspeech_alphabet)
			rospy.loginfo(self.deepspeech_beam_width)

			self.deepspeech_recognizer = Model(self.deepspeech_languageModel, self.deepspeech_n_features, self.deepspeech_n_context, self.deepspeech_alphabet, self.deepspeech_beam_width)
			rospy.loginfo("Deepspeech recognizer model initialized")
			self.deepspeech_recognizer.enableDecoderWithLM(self.deepspeech_alphabet, self.deepspeech_languageModel, self.deepspeech_trie, self.deepspeech_lm_alpha, self.deepspeech_lm_beta)
			rospy.loginfo("Successfully opened and configured recognizer")

		# using google speech recognition API
		elif(mode == "google"):
			rospy.logwarn("Google's speech recognition API chosen\nInitializing API parameters")

			rospy.loginfo("Initializing recognizer")
			self.google_recognizer = speech_recognition.Recognizer()
			rospy.loginfo("Successfully initialized recognizer")

			rospy.loginfo("Initializing microphone object")
			self.google_microphone = speech_recognition.Microphone(sample_rate = self.audio_samplerate)
			rospy.loginfo("Successfully initialized microphone object")

		# using CMU sphinx speech recognition API
		elif(mode == "sphinx"):
			rospy.logwarn("CMUSphinx API chosen\nInitializing API parameters")

			# Acoustic model directory path : please change it according to your directory location
			self.sphinx_acousticModel = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/sphinx/en-us"
			if not (os.path.exists(self.sphinx_acousticModel)):
				rospy.logerr("Acoustic model diretory does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()

			# Language model file path : please change it according to your file location
			self.sphinx_languageModel = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/sphinx/en-us.lm.bin"
			if not (os.path.isfile(self.sphinx_languageModel)):
				rospy.logerr("Langage model diretory does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()

			# Dictionnary file path : please change it according to your file location
			self.sphinx_dictionnary = "PATH TO YOUR CREATED PACKAGE/turtlebot_speech_recognition/audio_data/sphinx/cmudict-en-us.dict"
			if not (os.path.isfile(self.sphinx_dictionnary)):
				rospy.logerr("Dictionnary file does not exist, please check path")
				rospy.signal_shutdown("path error")
				sys.exit()
		pass

	#---------------------------------------------------------------------------------------------------------------------------------------
	# function to capture the sound from the microphone, process it through speech recognition and publish the recognized words as output	
	#---------------------------------------------------------------------------------------------------------------------------------------
	def runAudioTreatment(self, mode):
		# using deepspeech speech recognition API
		if(mode == "deepspeech"):
			# deepspeech only accepts PCM_16 wav file
			rospy.loginfo("Starting deepspeech audio analyzer")
			while not rospy.is_shutdown():
				print("\tSpeak !")
				self.audio_data = sounddevice.rec(int(self.audio_duration * self.audio_samplerate), samplerate = self.audio_samplerate, channels = 1)
				sounddevice.wait()  # Wait until recording is finished
				soundfile.write('deepspeech_voice.wav', self.audio_data, self.audio_samplerate, 'PCM_16', endian = None, format = 'WAV')

				# open wav file
				try:
					t = time.time()
					print("\n\tOpening WAV file")
					audio, fs = soundfile.read('deepspeech_voice.wav', frames =-1, start = 0, stop = None, dtype = 'int16')
					print("\tSuccessfully opened WAV file")
				except:
					rospy.logerr("cannot open WAV file, check path and if the file exist anymore")
					rospy.signal_shutdown("wav open error")

				# search for words/sentences and publish them
				if (fs == self.audio_samplerate):
					# use recognizer to find words / sentences in the wav file
					print("\trecognizing words or sentences in the wav file")
					result = self.deepspeech_recognizer.stt(audio, fs)

					if (result != "" and result != None):
						rospy.logwarn("words or sentences recognized from wav file : {}".format(result))
						self.stringPublisher.publish(result)
					else:
						rospy.logwarn("No words or sentences recognized in this wav file")
				else:
					rospy.logerr("\tWav file's sample frequency is not equal to sample rate, closing file")
					rospy.signal_shutdown("Sample rate error")
				print("\tAudio processed in {} seconds".format(time.time() - t))
			pass

		# using google speech recognition API
		elif(mode == "google"):
			rospy.loginfo("Starting google audio analyzer")
			while not rospy.is_shutdown():
				with self.google_microphone as source:
					t = time.time()
					try:
						print("\tSpeak !")
						# read input from standard microphone (phrase time limit : x sec)
						audio = self.google_recognizer.listen(source, phrase_time_limit = self.audio_duration)
						# Ask google speech Api to find words and get the result if it exists, then publish it
						result = self.google_recognizer.recognize_google(audio)
						print ("word recognized : " + result)
						self.stringPublisher.publish(result)

					except speech_recognition.UnknownValueError:
						print("\tGoogle Speech Recognition could not understand audio")

					except speech_recognition.RequestError as e:
						rospy.logerr("Could not request results from Google Speech Recognition service; {0}".format(e))
					print("\tAudio processed in {} seconds".format(time.time() - t))
			pass

		# using CMU sphinx speech recognition API
		elif(mode == "sphinx"):

			# initialize pocketsphinx
			rospy.loginfo("Initializing CMU Sphinx recognizer")
			config = Decoder.default_config()
			rospy.loginfo("Successfully initialized CMU Sphinx recognizer")

			# Hidden Markov model: The acoustic model which has been used
			config.set_string('-hmm', self.sphinx_acousticModel)
			# language model used
			config.set_string('-lm', self.sphinx_languageModel)
			# dictionary used
			config.set_string('-dict', self.sphinx_dictionnary)

			# Initialize decoder object
			sphinx_decoder = Decoder(config)

			# treatment with continuous stream : don't forget to plug the microphone into the PC jack and configure it
			# Go to Alsa (default linux audio setup panel) and choose your microphone as input audio
			# Pocketsphinx requires 16kHz, mono, 16-bit little-Endian audio.
			rospy.loginfo("Opening the audio channel")
			audio_object = pyaudio.PyAudio()
			sphinx_stream = audio_object.open(format=pyaudio.paInt16, channels=1, rate=self.audio_samplerate, input=True, frames_per_buffer=1024)
			sphinx_stream.start_stream()
			rospy.loginfo("Successfully opened the audio channel")

		
			while not rospy.is_shutdown():
				print("test")
				# start decoder
				sphinx_decoder.start_utt()

				while True:
					# read the input stream and if sound is detected, process the audio chunk through the decoder
					buf = sphinx_stream.read(2048)
					if buf:
						sphinx_decoder.process_raw(buf, False, False)
					else:
						break

					# publish the result if any word/sentence is found
					if sphinx_decoder.hyp() != None:
						self.stringPublisher.publish(sphinx_decoder.hyp().hypstr)
						print("string recognized : " + sphinx_decoder.hyp().hypstr)
						sphinx_decoder.end_utt()
						sphinx_decoder.start_utt()
			pass
		pass

	#-----------------------------------------------------------------
	# in case of forced stop, as CTRL + C, print an information message
	#-----------------------------------------------------------------
	def shutdown(self):
		rospy.logwarn("Closing audio treatment program")
		sys.exit()
		pass


#-----------------------------------------------------------------
# main function : used to start the program
#-----------------------------------------------------------------
if __name__ == "__main__":
	try:
		# start the program (run the init function from the main class)
		audio = audioTreatment()
	except Exception as e:
		rospy.logerr("an error: ({}) has occured, closing program".format(e))
	pass