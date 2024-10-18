#!/usr/bin/env python

import argparse
import base64
import datetime
import hashlib
import hmac
import json
import os
import time
import token
import uuid
from html import parser
from os.path import exists

import requests
import yaml

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Get SwitchBot devices list.")
    parser.add_argument("--token", type=str, help="SwitchBot API token", required=True)
    parser.add_argument(
        "--secret", type=str, help="SwitchBot API secret", required=True
    )
    args = parser.parse_args()

    token = args.token
    secret = args.secret

    nonce = str(uuid.uuid4())
    t = int(round(time.time() * 1000))
    string_to_sign = "{}{}{}".format(token, t, nonce)
    string_to_sign = bytes(string_to_sign, "utf-8")
    secret = bytes(secret, "utf-8")
    sign = base64.b64encode(
        hmac.new(secret, msg=string_to_sign, digestmod=hashlib.sha256).digest()
    )

    apiHeader = {}
    apiHeader["Authorization"] = token
    apiHeader["Content-Type"] = "application/json"
    apiHeader["charset"] = "utf8"
    apiHeader["t"] = str(t)
    apiHeader["sign"] = str(sign, "utf-8")
    apiHeader["nonce"] = nonce

    response = requests.get(
        "https://api.switch-bot.com/v1.1/devices", headers=apiHeader
    )
    response_jsontext = response.json()
    message = yaml.dump(response_jsontext, default_flow_style=False)
    print(message)

