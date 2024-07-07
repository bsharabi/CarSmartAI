import speech_recognition as sr
import pyaudio

v_command=''
speed_set = 80


p = pyaudio.PyAudio()
info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print ("Input Device id "+str(i)+" - "+p.get_device_info_by_host_api_device_index(0, i).get('name'))

def run():
    global v_command
    # obtain audio from the microphone
    r = sr.Recognizer()
    with sr.Microphone(device_index =1,sample_rate=48000) as source:
        r.record(source,duration=2)
        r.adjust_for_ambient_noise(source)
       
        print("Command?")
        audio = r.listen(source)
    try:
        v_command = r.recognize_sphinx(audio,
        keyword_entries=[('forward',1.0),('backward',1.0),
        ('left',1.0),('right',1.0),('stop',1.0)])        #You can add your own command here
        print(v_command)
      
    except sr.UnknownValueError:
        print("say again")
       
    except sr.RequestError as e:
        pass

    #print('pre')

run()


import speech_recognition as sr

r = sr.Recognizer()
m = sr.Microphone(2)

try:
    print("A moment of silence, please...")
    with m as source: r.adjust_for_ambient_noise(source)
    print("Set minimum energy threshold to {}".format(r.energy_threshold))
    while True:
        print("Say something!")
        with m as source: audio = r.listen(source)
        print("Got it! Now to recognize it...")
        try:
            # recognize speech using Google Speech Recognition
            value = r.recognize_google(audio)

            print("You said {}".format(value))
        except sr.UnknownValueError:
            print("Oops! Didn't catch that")
        except sr.RequestError as e:
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
except KeyboardInterrupt:
    pass