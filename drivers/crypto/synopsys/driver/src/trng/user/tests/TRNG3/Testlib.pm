package TRNG3::Testlib;

use strict;

use Getopt::Long;
use Pod::Find qw(pod_where);
use Pod::Usage;
use IO::File;

use Exporter;
our ($VERSION, @ISA, @EXPORT);

$VERSION = 0;
@ISA     = qw(Exporter);
@EXPORT  = qw();

our $device = "clp800";
our $result = undef;
our $quiet = 0;
our @handles;

sub print_version {
   print "trng3-tests 1.0\n";
}

sub print_help {
   print_version();
   pod2usage( { -verbose => 99,
                -exitval => "NOEXIT",
                -section => [ qw/SYNOPSIS OPTIONS/ ]} );
   pod2usage( { -input => pod_where({-inc => 1}, __PACKAGE__),
                -verbose => 99,
                -exitval => "NOEXIT",
                -section => [ qw/OPTIONS/ ]}, );
}

=head1 GLOBAL OPTIONS

=over 8

=item B<-d>, B<--device>=F<device>

Specify the platform device name (default: clp800).

=item B<-q>, B<--quiet>

Do not print PASSED on successful test runs.  Failure messages are still
printed.

=back

=cut
sub init {
   Getopt::Long::Configure('gnu_getopt', 'auto_help');
   GetOptions(
      'd|device=s' => \$device,
      'q|quiet'    => \$quiet,
      'V|version'  => sub { print_version(); $result = 0; exit },
      'H|help'     => sub { print_help();    $result = 0; exit },
      @_,
   ) or die;
}

sub pass {
   print STDOUT "PASSED\n" unless ($quiet);
   $result = 0;
   exit 0;
}

sub fail {
   my ($msg) = @_;

   chomp($msg);
   print STDOUT "FAILED: $msg\n";
   $result = 1;
   exit 1;
}

END {
   die "FAILED to complete test\n" unless defined $result;
}

sub sysfs_open {
   chdir "/sys/bus/platform/devices/$device"
      or die "failed to open $device: $!\n";

   # Put device into state suitable for testing
   my $mode = new IO::File(">mode")
      or die "failed to open mode: $!\n";

   $mode->printflush("promiscuous");
   $mode->close;
}

sub open_attribute {
   my ($filename, $mode) = @_;
   my %attr;

   $attr{filename} = $filename;
   $attr{handle} = new IO::File($filename, $mode)
      or die "failed to open $filename: $!\n";

   return push(@handles, (\%attr)) - 1;
}

sub read_attribute {
   my ($index) = @_;
   my $attr = $handles[$index];
   my ($handle, $filename);

   die "invalid handle: $index\n" unless defined($attr);

   $handle = $attr->{handle};
   $filename = $attr->{filename};

   $handle->seek(0, 0) or die "failed to rewind $filename: $!\n";
   $_ = <$handle> or die "failed to read $filename: $!\n";

   chomp;
   return $_;
}

sub write_attribute {
   my ($index, $data) = @_;
   my $attr = $handles[$index];
   my ($handle, $filename);

   die "invalid handle: $index\n" unless defined($attr);

   $handle = $attr->{handle};
   $filename = $attr->{filename};

   $handle->printflush($data)
      or die "failed to write $filename: $!\n";
}

1;
