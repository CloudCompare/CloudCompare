#!/usr/bin/perl

use File::Find;
@ARGV = ('.') unless @ARGV;

find(\&processOneFile, @ARGV);

sub processOneFile() {
    my $f = $File::Find::name;
    if ($f =~ /.*\.h$/ || $f =~ /.*\.cpp$/) {
        open(FILE,"<$f");
        @newcontent=();
        while (<FILE>) {
            if (/#include.*?\\.*/) {
                print "$_";
                s,\\,/,g;
                print "=> $_";
            }
            push @newcontent, $_;
        }
        close(FILE);
        open(FILE,">$f");
        print FILE @newcontent;
        close(FILE);
    }
}
