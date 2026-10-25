"""Offline Chinese WAV transcription. Never opens a microphone or network."""
import argparse
import json
import time
import wave

def transcribe(path,model_path='/ws/output/v2_brain/assets/vosk-model-small-cn-0.22'):
    from vosk import Model,KaldiRecognizer,SetLogLevel
    SetLogLevel(-1);start=time.monotonic();segments=[];confidence=[]
    with wave.open(str(path),'rb') as wav:
        if wav.getnchannels()!=1 or wav.getsampwidth()!=2 or wav.getcomptype()!='NONE':raise ValueError('Expected mono 16-bit PCM WAV')
        if not 8000<=wav.getframerate()<=48000 or wav.getnframes()/wav.getframerate()>30:raise ValueError('Invalid sample rate or audio exceeds 30 seconds')
        recognizer=KaldiRecognizer(Model(model_path),wav.getframerate());recognizer.SetWords(True)
        while data:=wav.readframes(4000):
            if recognizer.AcceptWaveform(data):segments.append(json.loads(recognizer.Result()))
        segments.append(json.loads(recognizer.FinalResult()))
    text=''.join(row.get('text','').replace(' ','') for row in segments)
    for row in segments:confidence.extend(w.get('conf',0.) for w in row.get('result',[]))
    return dict(text=text,source='offline_vosk_cn_0.22',mean_word_confidence=sum(confidence)/max(1,len(confidence)),
                latency_seconds=time.monotonic()-start,audio_uploaded=False)

def main():
    p=argparse.ArgumentParser();p.add_argument('wav');a=p.parse_args();print(json.dumps(transcribe(a.wav),ensure_ascii=False))

if __name__=='__main__':main()
