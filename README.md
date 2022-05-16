# Relays HTTPD Server

The project consists of HTTPD server with  URI handling of Relay control REST API.

## Compile

In order to compile, copy the example .vscode/settings_examples.json to .vscode/settings.json and configure the paths.
The ESP-IDF vscode plugin should also be installed

## Usage

Control network relay over HTTP GET request:

```
    http://<ip address>/out?<output number>=<state>
```

where <state> is

0=OFF
1=ON
2-1000 toggle (OFF->ON->OFF) the relay for specified time in milliseconds

### Repeat

For the cases when repeat is needed, then the following can be used:

```
    http://<ip address>/out?<output number>=<state>&repeat=<repeat number>
```



## Example

### Output toggle

```
    http://192.168.10.126/out?2=100
```

will toggle the relay OFF-ON-OFF for 100mS

### Repeat Output toggle


```
    http://192.168.10.126/out?2=100&repeat=3
```

will toggle the relay OFF-ON-OFF 3 times for 300mS every ON and 300mS between the repeats.

