![](https://www.plantuml.com/plantuml/svg/TP91QuGm48Nlyoi6zjn3Uzn3MRiiMrdHHRoL7iHCtK2C9Ob2_ts9TcGmzAZpvRqtBpdvqBem3GQQx8fFhvYLv_UgnYE-5NbvW_Kp3kByY7LMNlFoYdcPDv2SkLT50T1sGSzjeEN8eH-D0h1Z83Jq5S2DarNu36HL_0dmiOdxTgP50uDhhHzqmE01FZSBYJZgICyK2QbEt-P7gdCO3ddmsipaO2VRFKXDvaywEz-s102AM0EIEXWNK9oajh86f_i7z_32ITCrHHRH1jUrOqGTmeNx42uo9fSXH7kovE8uNlJb7sTkQmSb9IJlO-4bjaxItLTIB2M8nB--tAkwsMAajoGnE95Lqs1-0G00)

- the BLINK and RANGE INIT messages aren't part of the ranging procedure, just the initial setup
- POLL message communicates the transmit delay value used in subsequent messages
- POLL ACK, RANGE, RANGE REPORT are all transmitted with a delay transmit operation

### Build and upload commands

To build the STM32 based Bluepill board version, acting as TAG, run:

```sh
pio run -e bp_tag -t upload
```

If you want it to act as an ANCHOR run:

```sh
pio run -e bp_anchor -t upload
```
