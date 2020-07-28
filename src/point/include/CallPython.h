#ifndef CALL_PYTHON_H
#define CALL_PYTHON_H


class CallPython {
    public:
        CallPython(char *file_name, char *function, char *photo_path, float threshold);
        void execute();

    private:
        char *_file_name;
        char *_function;
        char *_photo_path;
        float _threshold;

};



#endif