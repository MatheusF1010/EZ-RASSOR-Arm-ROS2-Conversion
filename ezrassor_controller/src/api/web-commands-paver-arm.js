export default class HTTP { 

    static doPost(host, message) { 

        return fetch(
            host,
            {
                method: 'POST',
                headers:{
                    'Content-Type': 'application/json'
                },
                body: message
            }
        )
        .catch((error) => {
            console.log(error);
        });
    }
}