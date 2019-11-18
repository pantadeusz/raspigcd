# How to use interactive console

You can run gcd in the following way:

```bash
./gcd -c ../v3.json --configtest
```

## How to use interactive console with NodeJS

This is nodejs example

```js
const { spawn } = require('child_process');
const child = spawn('./gcd', ['-c', '../v3.json','--configtest']);

child.stdout.on('data', (data) => {
    console.log(`stdout: ${data}`);
  });
  
  child.stderr.on('data', (data) => {
    console.log(`stderr: ${data}`);
  });
  
//// since these are streams, you can pipe them elsewhere
//child.stderr.pipe(dest);

child.on('close', (code) => {
  console.log(`child process exited with code ${code}`);
});

child.stdin.write("go G0X100Y100\n");
child.stdin.write("q\n");
```
