var git=require('git-rev')
var squash=require('squash-node')
var sq=new squash();
git.long(function(rev){
sq.configure({
APIHost:'http://ec2-52-90-50-12.compute-1.amazonaws.com/',
APIKey:'83d7c88e-0a2b-4257-85d2-5a37f1afc968',
environment:'bokkale',
revision:rev
})
var err=new Error('test bug');
console.log(rev);
console.log(sq.options);
//console.log(err.stack);
sq.report(err);
})