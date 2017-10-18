#include "tokenizer.h"

Token::Token(){
    nPalabras = 0;
    tokenizado = false;
    linea[0] = '\0'; // Borramos la linea
    ficheroLeido = false;
    finFich = false;
    indice = 0;
    cuantosBuffer1 = cuantosBuffer2 = 0;
}

/*****************************************************************************/

Token::~Token (){
    destruyeToken();
}

/*****************************************************************************/

void Token::destruyeToken(){
    nPalabras = 0;
    linea[0] = '\0';
    tokenizado = false;
    nomFichero[0] = '\0';
    if(fichero.is_open()){
        fichero.close();
    }
    ficheroLeido = false;
    finFich = false;
    indice = 0;
    cuantosBuffer1 = cuantosBuffer2 = 0;
}

/*****************************************************************************/

int Token::abrirFichero(const char * f){
    try {
        fichero.open(f);
        if(!fichero) throw (f);
    } catch (const char * ){
        estado = noAbierto;
        return noAbierto;
    }
    strcpy(nomFichero, f);

    //Llenado de los buffers
    indice = 0;
    fichero.read(buffer, TAM_BUFFER);
    cuantosBuffer1 = fichero.gcount();
    ficheroLeido = (cuantosBuffer1 < TAM_BUFFER);
    if(!ficheroLeido){
       fichero.read(buffer+INICIO_BUFFER_2, TAM_BUFFER);
       cuantosBuffer2 = fichero.gcount();
       ficheroLeido = (cuantosBuffer2 < TAM_BUFFER);
    }

    estado = ok;
    finFich = (cuantosBuffer1 == 0);
    if(finFich)
       estado = eof;
    return ok;
}

/*****************************************************************************/

int Token::leeLinea(){

    int i;
    char c;

    if (!fichero.is_open()){
         estado = noAbierto;
         return noAbierto;
    }

    if (finFich){
       estado = eof;
       return eof;
    }

    linea[0] = '\0';
    tokenizado = false;

    /* Leemos lineas hasta que encontremos una que no este vacia, es
       decir que contenga al menos un caracter != '\n', o hasta eof
    */
    while( (!finFich) && (strlen(linea) == 0) ){
        i = 0;
        do{
           c = buffer[indice];
           if(c != 13){ // 13 == 0Dh == \n windows
              if (c == '\n')
                 linea[i] = 0;
              else
                 linea[i] = c;
              i++;
           }
           indice ++;
           if(indice == cuantosBuffer1){
              // BUFFER 1 VACIO => POSIBLE LLENADO DEL BUFFER 1
              if(indice < FIN_BUFFER1){
                 finFich = true;
              }
              if(!ficheroLeido){
                 fichero.read(buffer, TAM_BUFFER);
                 cuantosBuffer1 = fichero.gcount();
                 ficheroLeido = (cuantosBuffer1 < TAM_BUFFER);
              }
           }else if (indice == INICIO_BUFFER_2 + cuantosBuffer2){
              // BUFFER 2 VACIO => POSIBLE LLENADO DEL BUFFER 2
              if(indice < FIN_BUFFER2){
                 finFich = true;
              }
              if(!ficheroLeido){
                 fichero.read(buffer+INICIO_BUFFER_2, TAM_BUFFER);
                 cuantosBuffer2 = fichero.gcount();
                 ficheroLeido = (cuantosBuffer2 < TAM_BUFFER);
              }
              indice = 0;
           }
        }while(c != '\n' && !finFich);
    }

    nPalabras = 0;

    (strlen(linea) == 0 ? estado = eof : estado = ok);
    return estado;

}

/*****************************************************************************/

int Token::lee(int ncampos, char * palInicial){
    leeLinea();
    if(estado != ok) return estado;

    estado = ok;
    if ((tokenizar(" ") != ncampos)
        || (palInicial && (strcmp(token(0), palInicial) !=0)))
        estado = malFormato;
    return estado;
}

/*****************************************************************************/

int Token::tokenizar(const char * s){

   if (tokenizado) return nPalabras;

   strcpy(auxLinea, linea);

    unsigned int i, j; // Indices de las cadenas
    unsigned int lonAuxLinea; // Es de uso obligatorio ya que hay momentos
                              // en los que truncamos auxLinea poniendo
                              // caracteres a cero => strlen(auxLinea) cambia
    unsigned int lons; // Se utiliza por eficiencia, para no estar llamando
                       // todo el rato a strlen(s).

    palabras[nPalabras] = auxLinea;

    lonAuxLinea = strlen(auxLinea);
    lons = strlen(s);

    // Obtiene los tokens utilizando para delimitarlos los caracteres
    // de la cadena s
    for (i = 0; i < lonAuxLinea; i++){
       for (j = 0; j < lons; j++){
          if(s[j] == auxLinea[i]){
             auxLinea[i] = '\0';
             if (strlen(palabras[nPalabras]) > 0){
                nPalabras ++;
             }
             palabras[nPalabras] = auxLinea + i + 1;
             continue;
          }
      }
   }
   if (strlen(palabras[nPalabras]) > 0){
       nPalabras ++;
   }

   // Devolvemos el nº de tokens encontrados
   return nPalabras;
}

/*****************************************************************************/

char * Token::token(int n){
    if(n >= nPalabras) return NULL;
    return palabras[n];
}
