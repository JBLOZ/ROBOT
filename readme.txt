# Instrucciones para ejecutar el programa

## Requisitos

Asegúrate de tener instaladas las dependencias especificadas en el archivo `requirements.txt`. Puedes instalarlas ejecutando el siguiente comando:

```
pip install -r requirements.txt
```

## Ejecución del Programa

El programa puede ejecutarse en dos modos diferentes:

1. **Modo fuzzy (lógica difusa):**
   Ejecuta el siguiente comando en la terminal:
   ```
   python ./main.py fuzzy
   ```

2. **Modo experto (lógica no difusa):**
   Ejecuta el siguiente comando en la terminal:
   ```
   python ./main.py expert
   ```

Ambos comandos inicializan el robot y lo ponen en movimiento, empleando el modelo correspondiente.

### Nota

- Asegúrate de ejecutar el programa desde el directorio donde se encuentra el archivo `main.py`.
- Si tienes problemas con la versión de `pygame` o `fuzzy-expert`, prueba a instalar versiones compatibles con tu sistema.
