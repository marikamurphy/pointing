#include "point/CallPython.h"
#include <Python.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

//construct an object to call objectDetection
//TODO? we could try to generalize if we need to call another python file
CallPython::CallPython(char *file_name, char *function, char *photo_path, float threshold){
    _file_name = file_name;
    _function = function;
    _photo_path = photo_path;
    _threshold = threshold;

}

//execute the python module
//TODO: return the image with rectangles
void CallPython::execute(){
     
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pPhoto, *pThreshold, *pValue;
   
    Py_Initialize();
    //we need to get the path for the python file
    std::string abspath = "sys.path.append(\""+ boost::filesystem::current_path().string() + "/point/src\")";
    PyRun_SimpleString("import sys");
    PyRun_SimpleString(abspath.c_str());
    std::cout <<boost::filesystem::current_path().c_str() << std::endl;
    //pName = PyUnicode_DecodeFSDefault(_file_name);
    pName = PyString_FromString(_file_name);
    // Error checking of pName left out 
     
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    if (pModule == NULL)
    {
        PyErr_Print();
        std::exit(1);
    }

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, _function);
        // pFunc is a new reference 

        if (pFunc && PyCallable_Check(pFunc)) {
            //Get the arguments photo path and threshold
            pArgs = PyTuple_New(2); //generalize?
            pPhoto = PyLong_FromLong(atoi(_photo_path));
            if (!pPhoto) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert photo file path\n");
            }
            // pValue reference stolen here: 
            PyTuple_SetItem(pArgs, 0, pPhoto);
            pThreshold = PyLong_FromLong(_threshold);
            if (!pThreshold) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert threshold\n");
            }
            // pValue reference stolen here: 
            PyTuple_SetItem(pArgs, 1, pThreshold);
            //try to execute
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);

            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
            }
        }
        else {
                if (PyErr_Occurred())
                    PyErr_Print();
                fprintf(stderr, "Cannot find function \"%s\"\n", _function);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", _file_name);
    }
    
    Py_Finalize(); // errors during finalization ignored

} 