package main

import (
	"bytes"
	"fmt"
	"io"
	"log"
	"net/http"
	"strconv"
	"time"

	"encoding/json"

	"github.com/gorilla/mux"
	"gocv.io/x/gocv"
)

var imgwindow *gocv.Window
var previewFrameCounter int = 1

func get(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message": "get called"}`))
}

func post(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusCreated)
	w.Write([]byte(`{"message": "post called"}`))
}

func notFound(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusNotFound)
	w.Write([]byte(`{"message": "not found"}`))
}

//IsJSONString returns true if string s is a string
func IsJSONString(s string) bool {
	var js string
	return json.Unmarshal([]byte(s), &js) == nil

}

//IsJSON returns true if string s is a json string
func IsJSON(s string) bool {
	//var js map[string]interface{}
	var js json.RawMessage
	return json.Unmarshal([]byte(s), &js) == nil

}

func createImage(w http.ResponseWriter, request *http.Request) {
	err := request.ParseMultipartForm(32 << 20) // maxMemory 32MB
	if err != nil {
		w.WriteHeader(http.StatusBadRequest)
		return
	}
	//Access the photo key - First Approach
	file, h, err := request.FormFile("img_upload")
	defer file.Close()
	if err != nil {
		file.Close()
		w.WriteHeader(http.StatusBadRequest)
		return
	}

	//debug
	log.Println("createImage : Received image = " + h.Filename + strconv.Itoa(previewFrameCounter) + ".jpg")

	buf := bytes.NewBuffer(nil)
	if _, err := io.Copy(buf, file); err != nil {
		file.Close()
		log.Println("createImage : image corrupted ")
	}
	cvMat, err := gocv.NewMatFromBytes(300, 300, 16, buf.Bytes())
	if err != nil {
		file.Close()
		log.Println("createImage : opening image failed ")
	}
	imgwindow.IMShow(cvMat)
	//file.Close()

	/* tmpfile, err := os.Create("./" + h.Filename + strconv.Itoa(previewFrameCounter) + ".jpg")
	previewFrameCounter++
	defer tmpfile.Close()
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		return
	}
	_, err = io.Copy(tmpfile, file)
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		return
	}
	tmpfile.Close() */

	w.WriteHeader(200)
	return
}

func carvisionImageUpload(w http.ResponseWriter, r *http.Request) {
	pathParams := mux.Vars(r)
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Access-Control-Allow-Origin", "*")

	var err error
	userID := ""

	//debug
	log.Println("carvisionImageUpload : Received URI = " + r.RequestURI)

	if val, ok := pathParams["userID"]; ok {
		userID = val
	} else {
		w.WriteHeader(http.StatusBadRequest)
		w.Write([]byte(`{"message": "need a UserID"}`))
		return
	}

	jsonLoad := ""
	if val, ok := pathParams["jsonLoad"]; ok {
		if IsJSON(val) {
			jsonLoad = val
		} else {
			w.WriteHeader(http.StatusBadRequest)
			w.Write([]byte(`{"message": "need data in json format"}`))
			return
		}
	} else {
		w.WriteHeader(http.StatusBadRequest)
		w.Write([]byte(`{"message": "need data"}`))
		return
	}

	query := r.URL.Query()
	locationLat := query.Get("lat")
	locationLng := query.Get("lng")
	lat := 0.0
	lng := 0.0
	lat, err = strconv.ParseFloat(locationLat, 32)
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		w.Write([]byte(`{"message": "need a number for latitude"}`))
		return
	}
	lng, err = strconv.ParseFloat(locationLng, 32)
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		w.Write([]byte(`{"message": "need a number for longitude "}`))
		return
	}

	fmt.Printf(" %s %s %v %v \n", userID, jsonLoad, lat, lng)

	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message": "all ok "}`))
	return

}

func main() {
	httpPort := 8080

	fmt.Printf("hello, world, on %v \n", httpPort)

	fmt.Println("The time is", time.Now())

	imgwindow = gocv.NewWindow("Hello")

	//s := &server{}
	//http.HandleFunc("/", home)
	/*
		r.HandleFunc("/", get).Methods(http.MethodGet)
		r.HandleFunc("/", post).Methods(http.MethodPost)
		r.HandleFunc("/", put).Methods(http.MethodPut)
		r.HandleFunc("/", delete).Methods(http.MethodDelete)
		r.HandleFunc("/", notFound)
	*/
	//log.Fatal(http.ListenAndServe(":8081", nil))

	r := mux.NewRouter()

	api := r.PathPrefix("/carvision").Subrouter()
	api.HandleFunc("", get).Methods(http.MethodGet)
	api.HandleFunc("", post).Methods(http.MethodPost)

	//http://127.0.0.1:8080/api/v1/user/1/comment/1?location=home
	//http: //127.0.0.1:8080/api/v1/user/1/comment/1
	//api.HandleFunc("/user/{userID}/comment/{commentID}", params).Methods(http.MethodGet)

	api.HandleFunc("/imageupload", createImage).Methods(http.MethodPost)

	//http://127.0.0.1:8080/api/v1/user/8dcbe0b68/token/e4ivZ01gQniqyW7XQCmZPS
	//api.HandleFunc("/user/{userID}/token/{token}", handleStoreTokenForUser).Methods(http.MethodPost)
	//http://127.0.0.1:8080/api/v1/user/8dcbe0b68/reason/TRIGGER_LU_RECEIVED
	//api.HandleFunc("/user/{userID}/reason/{reason}", handleStorePingForUser).Methods(http.MethodPost)

	log.Fatal(http.ListenAndServe("10.42.0.1:"+strconv.Itoa(httpPort), r))
}
