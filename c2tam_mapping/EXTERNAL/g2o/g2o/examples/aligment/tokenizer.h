#ifndef __TOKENIZER_H_
#define __TOKENIZER_H_


 // 1 MB de buffer y tenemos 2 buffers
#define TAM_BUFFER 1024 * 1024
#define INICIO_BUFFER_2 TAM_BUFFER
#define FIN_BUFFER1 TAM_BUFFER
#define FIN_BUFFER2 2*TAM_BUFFER

#define MAX_PALABRAS 800
#define TAM_LINEA 10000

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
using namespace std;

class Token
{
private:
    ifstream fichero; // Fichero desde el cual leeremos las lineas

    char buffer[2 * TAM_BUFFER]; // Es como 2 buffers de 1 MB
    int cuantosBuffer1, cuantosBuffer2; // cantidad de datos que hay en
                                        // cada buffer
    int indice; // indice del buffer (posicion en el fichero +/-)
    bool ficheroLeido; // Si ya esta leido todo el fichero y metido en los
                       // buffers
    bool finFich; // Ya esta todo el fichero leido y hemos vaciado los buffers

    char linea[TAM_LINEA]; // ultima linea leida del ficheo

    bool tokenizado; // indica si la ultima linea leida ha sido tokenizada

    char auxLinea[TAM_LINEA]; // copia de linea que se utiliza para obtener los
	                          // tokens o palabras de la linea

    int nPalabras; // nº de tokens encontrados en la linea tras la tokenizacion

    char * palabras[MAX_PALABRAS]; // tokens encontrados

public:
    enum {ok = 0, eof = -1, noAbierto = -2, errorLectura = -3, malFormato = -4};
    char nomFichero[50];
    int estado;

    /* Constructor */
    Token ();

    /* Destructor */
    ~Token ();

    void destruyeToken();

    int abrirFichero(const char * f);
    /* Devuelve: Si exito => true
        Sino false
    */

    void cerrar(){
        if (fichero && fichero.is_open()) fichero.close();
    }

    inline void reposicionarFichero(int off = 0){
        //fichero.seekg (off, ios::beg);
        indice = off;
    }

    inline int getOffset(){
        //return fichero.tellg();
        return indice;
    }

    int leeLinea();
    /* Descripcion: lee una línea del fichero de texto
       Devuelve: Si exito => ok
                 Si fin de fichero => eof
                 Si fichero no abierto => noAbierto
    */


    int lee(int ncampos, char * palInicial = NULL);
    /* Descripcion: lee una linea cumpliendo un cierto formato
       Devuelve: ok si todo va bien
                 eof si fin de fichero
                 malFormato si se ha leido bien la linea pero
                            no cumple con el formato
    */

    int tokenizar(const char * s);
    /* Descripcion: obtiene los tokens de la linea leida
                    para separar los tokens se utiliza la cadena s,
                    si un caracter de esta cadena coincide con alguno
                    de la linea entonces eso es un fin de token
       Devuelve: el nº de tokens encontrados
    */

    /* token (n) permite obtener el token que ocupa la posicion n
       en la línea */
    char * token(int n);
    /* Descripcion: obtiene el token n-esimo de la linea leida, en caso
                    de que no exista => NULL
    */

    int getnPalabras(){
        return nPalabras;
    }
    /* Descripcion: obtiene el nº de tokes de la ultima linea tokenizada */

    inline char * getLinea(){
        return (linea);
    }
    /* Descripcion: obtiene la ultima linea leida */

    inline bool finFichero(){
        return (finFich);
    }
};

#endif
