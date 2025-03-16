# RP2350_i2s
Polyphonic Synthesizer based on Raspberry Pi Pico 2 and i2s DAC module


# Manual
El proyecto a importar con el SDK de Raspberry Pi Pico está dentro de la carpeta `program`. Para ejecutar el `cmake` se debe:
- Crear un directorio de `build` y situarse en él: `>>mkdir build`, `>>cd build`
- Ejecutar el `cmake` del proyecto (que a su vez ejecutará el de las librerías incluídas): `>>cmake -G Ninja ..`
- El proyecto ya está listo para ser compilado y cargado a una Raspberry Pico 2 (microcontrolador RP2350)  
