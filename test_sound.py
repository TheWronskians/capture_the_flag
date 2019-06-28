from playsound import playsound
import time
if __name__ == "__main__":
    # time_rocky = time.time() + 5
    # while(time.time() < time_rocky):
        # playsound('music/rocky.mp3', block=True)
    # time_john_cena = time.time() + 5
    # while(time.time() < time_john_cena):
    #     playsound('music/john_cena.mp3')
    time_game_song = time.time() + 5
    while(time.time() < time_game_song):
        playsound('music/eye_of_the_tiger.mp3', block=True).pause()
    # playsound(None, windsound.SND_FILENAME)
