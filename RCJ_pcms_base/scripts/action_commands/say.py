#!/usr/bin/env python3
import argparse
import pyttsx3
import sys

engine = pyttsx3.init()


def main(args):
    global engine
    engine.say(args['text'])
    engine.runAndWait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-t', '--text', type=str, required=True, help='Text you want the computer to say')
    parser.add_argument('-r', '--rate', type=int, default=130, help='Speed of the speaker')
    parser.add_argument('-v', '--volume', type=float, default=1.0, help='Volume of the speaker')
    parser.add_argument('-l', '--language', type=str, default='en-us', help='Language of the speaker')

    args = vars(parser.parse_args())

    engine.setProperty('rate', args['rate'])
    engine.setProperty('volume', args['volume'])
    engine.setProperty('voice', args['language'])

    try:
        main(args)
        sys.exit(0)
    except Exception as e:
        print(f'Program ended due to: {e}')
        sys.exit(1)
