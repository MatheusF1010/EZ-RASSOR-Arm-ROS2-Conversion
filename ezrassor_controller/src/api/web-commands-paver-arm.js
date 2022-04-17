export default class HTTP { 

    static doPost(host, message) { 

        return fetch(
            host,
            {
                //headers: {"Content-Type":"text/plain; charset=utf-8"},
                method: 'POST',
                headers:{
                    'Content-Type': 'application/json'
                },
                body: message
            }
        )
        .catch((error) => {
            console.log("Host: " + host + "\n");
            console.log("Message: " + message + "\n");
            console.log(error);
        });
    }
}